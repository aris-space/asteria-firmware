use crate::{KellerSensError, KellerSensRS485};

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BasicPressure {
    pub keller_id: u8,
}

impl BasicPressure {
    pub fn get_id(&self) -> u8 {
        self.keller_id
    }

    pub async fn get_pressure<'a>(
        &self,
        handle: &mut KellerSensRS485<'a>,
    ) -> Result<f32, KellerSensError> {
        handle.read_p1(self.get_id()).await
    }

    pub async fn get_pressure_temperature<'a>(
        &self,
        handle: &mut KellerSensRS485<'a>,
    ) -> Result<(f32, f32), KellerSensError> {
        handle.read_p1_tob1(self.get_id()).await
    }
}
pub struct DifferentialPressureLevel {
    pub keller_id: u8,
    pub zero_point: f32,
    pub full_point: f32,
}

impl DifferentialPressureLevel {
    pub fn get_id(&self) -> u8 {
        self.keller_id
    }

    pub async fn get_pressure<'a>(
        &self,
        handle: &mut KellerSensRS485<'a>,
    ) -> Result<f32, KellerSensError> {
        let p = handle.read_p1(self.get_id()).await?;
        Ok(p)
    }

    pub async fn get_level<'a>(
        &self,
        handle: &mut KellerSensRS485<'a>,
    ) -> Result<u8, KellerSensError> {
        let p = handle.read_p1(self.get_id()).await?;

        let lvl = (100.0 * (p - self.zero_point) / (self.full_point - self.zero_point)) as u8;

        Ok(lvl)
    }
}
