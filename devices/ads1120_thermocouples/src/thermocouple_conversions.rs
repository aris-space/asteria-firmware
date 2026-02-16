use micromath::F32Ext;

const COEFF_TYPE_K_NEGATIVE: [f32; 11] = [
    0.0,
    3.945_012_8E-2,
    2.362_237_3E-5,
    -3.285_890_7E-7,
    -4.990_482_7E-9,
    -6.750_906E-11,
    -5.741_032_6E-13,
    -3.108_887_3E-15,
    -1.045_160_9E-17,
    -1.988_926_7E-20,
    -1.632_269_8E-23,
];

const COEFF_TYPE_K_POSITIVE: [f32; 10] = [
    -1.760_041_3E-2,
    3.892_120_3E-2,
    1.855_877E-5,
    -9.945_759_4E-8,
    3.184_094_7E-10,
    -5.607_284_4E-13,
    5.607_506E-16,
    -3.202_072E-19,
    9.715_115E-23,
    -1.210_472_16E-26,
];

pub trait ThermocoupleConversion {
    fn voltage_to_temperature(&self, tc_voltage: f32, cj_voltage: f32) -> f32;
    fn temperature_to_voltage(&self, temp: f32) -> f32;
}

pub enum ThermocoupleType {
    K,
    J,
}
impl ThermocoupleConversion for ThermocoupleType {
    fn voltage_to_temperature(&self, tc_voltage: f32, cj_voltage: f32) -> f32 {
        match self {
            ThermocoupleType::K => voltage_to_temperature_k(tc_voltage + cj_voltage),
            ThermocoupleType::J => unimplemented!(),
        }
    }

    fn temperature_to_voltage(&self, temp: f32) -> f32 {
        match self {
            ThermocoupleType::K => temperature_to_voltage_k(temp),
            ThermocoupleType::J => unimplemented!(),
        }
    }
}

// Coefficients and implementation taken from richardeoin/thermocouple crate
fn temperature_to_voltage_k(temp: f32) -> f32 {
    // Convert temperature in Celsius to voltage in mV
    match temp > 0.0 {
        false => {
            // -270°C -> 0°C
            // Power Series
            let mut result = 0.0;

            for (i, &coeff) in COEFF_TYPE_K_NEGATIVE.iter().enumerate() {
                result += coeff * temp.powi(i as i32);
            }
            result
        }
        _ => {
            // 0°C -> 1372°C
            let a0 = 1.185_976E-1;
            let a1 = -1.183_432E-4;
            let a2 = 1.269_686E2;

            // Power Series
            let mut result = 0.0;

            for (i, &coeff) in COEFF_TYPE_K_POSITIVE.iter().enumerate() {
                result += coeff * temp.powi(i as i32);
            }

            // Exponential
            let es = a0 * (a1 * (temp - a2) * (temp - a2)).exp();

            result + es
        }
    }
}

fn voltage_to_temperature_k(voltage: f32) -> f32 {
    let coefficients: [f32; 10] = match (voltage < 0.0, voltage < 20.644) {
        (true, _) => [
            0.0000000E+00,
            2.5173462E+01,
            -1.1662878E+00,
            -1.0833638E+00,
            -8.977_354E-01,
            -3.7342377E-01,
            -8.663_265E-02,
            -1.0450598E-02,
            -5.1920577E-04,
            0.0000000E+00,
        ],
        (false, true) => [
            0.000000E+00,
            2.508355E+01,
            7.860106E-02,
            -2.503131E-01,
            8.315_27E-02,
            -1.228034E-02,
            9.804036E-04,
            -4.413_03E-05,
            1.057734E-06,
            -1.052755E-08,
        ],
        (false, false) => [
            -1.318058E+02,
            4.830222E+01,
            -1.646031E+00,
            5.464731E-02,
            -9.650715E-04,
            8.802193E-06,
            -3.110_81E-08,
            0.000000E+00,
            0.000000E+00,
            0.000000E+00,
        ],
    };

    // Power Series
    let mut result = 0.0;

    for (i, &coeff) in coefficients.iter().enumerate() {
        result += coeff * voltage.powi(i as i32);
    }
    result
}
