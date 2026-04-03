use core::fmt::Write as _;

use defmt_brtt::DefmtConsumer;

use super::{FileStorage, session_index};

fn session_dir_path(index: u32) -> Option<heapless::String<32>> {
    let mut path = heapless::String::<32>::new();
    write!(path, "/log_{index}").ok()?;
    Some(path)
}

fn session_file_path(index: u32, filename: &str) -> Option<heapless::String<64>> {
    let dir = session_dir_path(index)?;
    let mut path = heapless::String::<64>::new();
    write!(path, "{dir}/{filename}").ok()?;
    Some(path)
}

pub(super) fn active_session_file_path(filename: &str) -> Option<heapless::String<64>> {
    session_file_path(session_index()?, filename)
}

pub(super) fn default_log_path() -> heapless::String<64> {
    let mut path = heapless::String::<64>::new();
    let _ = path.push_str("/defmt.bin");
    path
}

fn next_session_index(files: &dyn FileStorage) -> u32 {
    let mut max_index: Option<u32> = None;
    let mut visit = |name: &str| {
        if let Some(suffix) = name.strip_prefix("log_")
            && let Ok(index) = suffix.parse::<u32>()
        {
            max_index = Some(max_index.map_or(index, |current| current.max(index)));
        }
    };
    let _ = files.read_dir("/", &mut visit);
    max_index.map_or(0, |index| index + 1)
}

fn write_build_info(files: &dyn FileStorage, session: u32) {
    use crate::built;

    let Some(path) = session_file_path(session, "build_info.txt") else {
        return;
    };

    let mut buf = heapless::String::<512>::new();
    let _ = write!(
        buf,
        "pkg={}\nprofile={}\ntarget={}\ngit={}\ndirty={}\nfeatures={}\n",
        built::PKG_NAME,
        built::PROFILE,
        built::TARGET,
        built::GIT_COMMIT_HASH_SHORT.unwrap_or("none"),
        match built::GIT_DIRTY {
            Some(true) => "true",
            Some(false) => "false",
            None => "none",
        },
        built::FEATURES_LOWERCASE_STR,
    );

    let _ = files.write_file(&path, buf.as_bytes());
}

pub(super) fn prepare_session(files: &dyn FileStorage) -> Option<u32> {
    let session = next_session_index(files);
    let dir = session_dir_path(session)?;
    files.create_dir(&dir).then_some(())?;
    write_build_info(files, session);
    defmt::info!("storage: session {}", dir.as_str());
    Some(session)
}

const STAGING_SIZE: usize = 512;

pub(super) struct DefmtStaging {
    buf: [u8; STAGING_SIZE],
    len: usize,
}

impl DefmtStaging {
    pub(super) fn new() -> Self {
        Self {
            buf: [0u8; STAGING_SIZE],
            len: 0,
        }
    }

    pub(super) fn stage(&mut self, files: &dyn FileStorage, path: &str, data: &[u8]) {
        if self.len + data.len() > STAGING_SIZE {
            self.flush(files, path);
        }

        let count = data.len().min(STAGING_SIZE - self.len);
        self.buf[self.len..self.len + count].copy_from_slice(&data[..count]);
        self.len += count;
    }

    pub(super) fn flush(&mut self, files: &dyn FileStorage, path: &str) {
        if self.len == 0 {
            return;
        }

        let _ = files.append_file(path, &self.buf[..self.len]);
        self.len = 0;
    }

    pub(super) fn drain(
        &mut self,
        files: &dyn FileStorage,
        path: &str,
        consumer: &mut DefmtConsumer,
    ) {
        while let Ok(grant) = consumer.read() {
            let data = grant.buf();
            let len = data.len();
            self.stage(files, path, data);
            grant.release(len);
        }

        self.flush(files, path);
    }
}
