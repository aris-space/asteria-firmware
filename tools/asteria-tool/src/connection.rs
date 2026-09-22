use std::fmt::{self, Write};
use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use clap::ValueEnum;
use nusb::DeviceInfo;
use postcard_rpc::header::VarSeqKind;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::{ERROR_PATH, WireError};
use wire_types::{FsInfoEndpoint, FsInfoReq};

const OUTGOING_DEPTH_DEFAULT: usize = 64;
const RAW_RETRIES: usize = 6;
const RAW_RETRY_DELAY_MS: u64 = 250;
const RAW_RPC_PROBE_TIMEOUT_MS_DEFAULT: u64 = 200;
const RAW_AUTOCONNECT_MANUFACTURER: &str = "ARIS";
#[cfg(not(target_os = "windows"))]
const RAW_INTERFACE_CLASS_VENDOR_SPECIFIC: u8 = 0xFF;
type RawConnectInfo = (
    HostClient<WireError>,
    Option<String>,
    Option<String>,
    Option<String>,
);
type RawConnectResult = std::result::Result<RawConnectInfo, RawConnectError>;

#[derive(Debug)]
enum RawConnectError {
    Retryable(anyhow::Error),
    Fatal(anyhow::Error),
}

impl RawConnectError {
    fn retryable(err: anyhow::Error) -> Self {
        Self::Retryable(err)
    }

    fn fatal(err: anyhow::Error) -> Self {
        Self::Fatal(err)
    }

    fn into_anyhow(self) -> anyhow::Error {
        match self {
            Self::Retryable(err) | Self::Fatal(err) => err,
        }
    }
}

struct RawCandidate {
    info: DeviceInfo,
    descriptor: RawDeviceDescriptor,
}

#[derive(Clone, Debug)]
struct RawDeviceDescriptor {
    manufacturer: Option<String>,
    product: Option<String>,
    serial: Option<String>,
    vendor_id: u16,
    product_id: u16,
}

impl RawDeviceDescriptor {
    fn from_info(info: &DeviceInfo) -> Self {
        Self {
            manufacturer: info.manufacturer_string().map(str::to_string),
            product: info.product_string().map(str::to_string),
            serial: info.serial_number().map(str::to_string),
            vendor_id: info.vendor_id(),
            product_id: info.product_id(),
        }
    }

    fn is_aris_device(&self) -> bool {
        self.manufacturer.as_deref() == Some(RAW_AUTOCONNECT_MANUFACTURER)
    }

    fn short_name(&self) -> String {
        let mut out = String::new();
        if let Some(manufacturer) = &self.manufacturer
            && !manufacturer.is_empty()
        {
            out.push_str(manufacturer);
        }
        if let Some(product) = &self.product
            && !product.is_empty()
        {
            if !out.is_empty() {
                out.push(' ');
            }
            out.push_str(product);
        }
        if out.is_empty() {
            let _ = write!(out, "USB {:04x}:{:04x}", self.vendor_id, self.product_id);
        }
        if let Some(serial) = &self.serial
            && !serial.is_empty()
        {
            let _ = write!(out, " (sn:{serial})");
        }
        out
    }

    fn option_line(&self) -> String {
        let mut out = self.short_name();
        let _ = write!(
            out,
            " [vid:{:04x} pid:{:04x}]",
            self.vendor_id, self.product_id
        );
        out
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum TransportMode {
    Auto,
    Raw,
    Serial,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ConnectedTransport {
    RawUsb {
        manufacturer: Option<String>,
        product: Option<String>,
        serial: Option<String>,
    },
    Serial(String),
}

impl ConnectedTransport {
    pub fn label(&self) -> String {
        match self {
            Self::RawUsb { .. } => "raw-usb".to_string(),
            Self::Serial(port) => format!("serial:{port}"),
        }
    }

    pub fn board_name(&self) -> Option<&str> {
        match self {
            Self::RawUsb { product, .. } => product.as_deref(),
            Self::Serial(_) => None,
        }
    }

    pub fn device_name(&self) -> Option<String> {
        match self {
            Self::RawUsb {
                manufacturer,
                product,
                serial,
            } => {
                let mut out = String::new();
                if let Some(m) = manufacturer {
                    out.push_str(m);
                }
                if let Some(p) = product {
                    if !out.is_empty() {
                        out.push(' ');
                    }
                    out.push_str(p);
                }
                if let Some(s) = serial
                    && !s.is_empty()
                {
                    if !out.is_empty() {
                        out.push(' ');
                    }
                    out.push_str("(sn:");
                    out.push_str(s);
                    out.push(')');
                }
                if out.is_empty() { None } else { Some(out) }
            }
            Self::Serial(_) => None,
        }
    }
}

impl fmt::Display for ConnectedTransport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.label())
    }
}

pub async fn connect(
    mode: TransportMode,
    port: Option<&str>,
    baud: u32,
) -> Result<(HostClient<WireError>, ConnectedTransport)> {
    match mode {
        TransportMode::Raw => connect_raw_with_retries(RAW_RETRIES)
            .await
            .map(raw_connected_transport)
            .map_err(RawConnectError::into_anyhow),
        TransportMode::Serial => {
            let port = port.context("serial mode requires --port <PATH>")?;
            let client = connect_serial(port, baud)?;
            Ok((client, ConnectedTransport::Serial(port.to_string())))
        }
        TransportMode::Auto => match connect_raw_with_retries(RAW_RETRIES).await {
            Ok(info) => Ok(raw_connected_transport(info)),
            Err(RawConnectError::Fatal(err)) => Err(err),
            Err(RawConnectError::Retryable(raw_err)) => {
                if let Some(port) = port {
                    let client = connect_serial(port, baud)?;
                    Ok((client, ConnectedTransport::Serial(port.to_string())))
                } else {
                    Err(anyhow!(
                        "failed to connect in auto mode over raw USB: {raw_err}. \
                             Provide --port for serial fallback."
                    ))
                }
            }
        },
    }
}

fn connect_serial(port: &str, baud: u32) -> Result<HostClient<WireError>> {
    HostClient::<WireError>::try_new_serial_cobs(
        port,
        ERROR_PATH,
        outgoing_depth(),
        baud,
        seq_kind(),
    )
    .map_err(|e| anyhow!("serial connection failed for {port}: {e}"))
}

fn raw_connected_transport(
    (client, manufacturer, product, serial): RawConnectInfo,
) -> (HostClient<WireError>, ConnectedTransport) {
    (
        client,
        ConnectedTransport::RawUsb {
            manufacturer,
            product,
            serial,
        },
    )
}

async fn connect_raw_with_retries(retries: usize) -> RawConnectResult {
    let mut last_retryable = None;
    for attempt in 0..retries {
        match connect_raw_once().await {
            Ok(c) => return Ok(c),
            Err(RawConnectError::Fatal(err)) => return Err(RawConnectError::Fatal(err)),
            Err(RawConnectError::Retryable(err)) => {
                last_retryable = Some(err);
            }
        }
        if attempt + 1 < retries {
            tokio::time::sleep(Duration::from_millis(RAW_RETRY_DELAY_MS)).await;
        }
    }

    Err(RawConnectError::retryable(anyhow!(
        "raw USB connection failed after {retries} attempts: {}",
        last_retryable
            .map(|e| e.to_string())
            .unwrap_or_else(|| "unknown error".to_string())
    )))
}

async fn connect_raw_once() -> RawConnectResult {
    let candidates = discover_raw_candidates()?;
    let (mut aris_candidates, _other_candidates): (Vec<_>, Vec<_>) = candidates
        .into_iter()
        .partition(|c| c.descriptor.is_aris_device());

    if aris_candidates.len() > 1 {
        return Err(RawConnectError::fatal(anyhow!(
            format_multiple_candidates_error(
                &aris_candidates,
                &format!("manufacturer \"{RAW_AUTOCONNECT_MANUFACTURER}\"")
            )
        )));
    }

    if let Some(candidate) = aris_candidates.pop() {
        return connect_and_validate_candidate(candidate)
            .await
            .map_err(RawConnectError::retryable);
    }

    Err(RawConnectError::retryable(anyhow!(
        "no USB device with manufacturer \"{RAW_AUTOCONNECT_MANUFACTURER}\" found"
    )))
}

fn discover_raw_candidates() -> std::result::Result<Vec<RawCandidate>, RawConnectError> {
    let devices = nusb::list_devices()
        .map_err(|e| RawConnectError::retryable(anyhow!("USB enumeration failed: {e:?}")))?;

    let mut out = Vec::new();
    for info in devices {
        let descriptor = RawDeviceDescriptor::from_info(&info);
        out.push(RawCandidate { info, descriptor });
    }
    Ok(out)
}

fn connect_candidate(candidate: RawCandidate) -> Result<RawConnectInfo> {
    let interface_id = select_interface_id(&candidate.info, &candidate.descriptor)
        .map_err(RawConnectError::into_anyhow)?;
    let client = HostClient::<WireError>::try_from_nusb_and_interface(
        &candidate.info,
        interface_id,
        ERROR_PATH,
        outgoing_depth(),
        seq_kind(),
    )
    .map_err(|e| {
        anyhow!(
            "failed to connect to {}: {e}",
            candidate.descriptor.short_name()
        )
    })?;

    Ok((
        client,
        candidate.descriptor.manufacturer,
        candidate.descriptor.product,
        candidate.descriptor.serial,
    ))
}

async fn connect_and_validate_candidate(candidate: RawCandidate) -> Result<RawConnectInfo> {
    let label = candidate.descriptor.short_name();
    let info = connect_candidate(candidate)?;
    validate_asteria_device(&info.0)
        .await
        .map_err(|e| anyhow!("device {label} rejected: {e}"))?;
    Ok(info)
}

async fn validate_asteria_device(client: &HostClient<WireError>) -> Result<()> {
    let timeout_ms = raw_probe_timeout_ms();
    let probe = client.send_resp::<FsInfoEndpoint>(&FsInfoReq);
    let result = tokio::time::timeout(Duration::from_millis(timeout_ms), probe)
        .await
        .map_err(|_| anyhow!("Asteria RPC probe timed out after {timeout_ms} ms"))?;
    result.map_err(|e| anyhow!("Asteria RPC probe failed: {e:?}"))?;
    Ok(())
}

fn format_multiple_candidates_error(candidates: &[RawCandidate], reason: &str) -> String {
    let mut msg = format!(
        "multiple matching USB devices found ({reason}); auto-connect is disabled.\n\
         Disconnect extra devices and retry.\nCandidates:"
    );

    for (idx, candidate) in candidates.iter().enumerate() {
        let _ = write!(
            msg,
            "\n  {}. {}",
            idx + 1,
            candidate.descriptor.option_line()
        );
    }
    msg
}

#[cfg(not(target_os = "windows"))]
fn select_interface_id(
    info: &DeviceInfo,
    descriptor: &RawDeviceDescriptor,
) -> std::result::Result<usize, RawConnectError> {
    info.interfaces()
        .position(|i| i.class() == RAW_INTERFACE_CLASS_VENDOR_SPECIFIC)
        .ok_or_else(|| {
            RawConnectError::retryable(anyhow!(
                "device {} has no interface with class 0x{RAW_INTERFACE_CLASS_VENDOR_SPECIFIC:02x}",
                descriptor.short_name()
            ))
        })
}

#[cfg(target_os = "windows")]
fn select_interface_id(
    _info: &DeviceInfo,
    _descriptor: &RawDeviceDescriptor,
) -> std::result::Result<usize, RawConnectError> {
    Ok(0)
}

fn outgoing_depth() -> usize {
    std::env::var("ASTERIA_OUTGOING_DEPTH")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .filter(|v| *v > 0)
        .unwrap_or(OUTGOING_DEPTH_DEFAULT)
}

fn raw_probe_timeout_ms() -> u64 {
    std::env::var("ASTERIA_RAW_PROBE_TIMEOUT_MS")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .filter(|v| *v > 0)
        .unwrap_or(RAW_RPC_PROBE_TIMEOUT_MS_DEFAULT)
}

fn seq_kind() -> VarSeqKind {
    match std::env::var("ASTERIA_SEQ_BYTES").ok().as_deref() {
        Some("1") => VarSeqKind::Seq1,
        Some("4") => VarSeqKind::Seq4,
        _ => VarSeqKind::Seq2,
    }
}
