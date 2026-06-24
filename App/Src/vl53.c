#include "vl53.h"
#include "stdio.h"

I2C_HandleTypeDef *hi2c = NULL;
uint8_t stop_variable;
uint32_t measurement_timing_budget_us;

// ---> NEW DYNAMIC ADDRESS TRACKER <---
uint8_t vl53_current_addr = (VL53L0X_ADDR << 1);

void vl53_set_target(uint8_t address_8bit) {
    vl53_current_addr = address_8bit;
}

#define DEVICE_TIMEOUT 500
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * ((uint32_t)vcsel_period_pclks) * 1655) + 500) / 1000)

void set_vl53_i2c_handler(I2C_HandleTypeDef *i2chandler){
	hi2c = i2chandler;
}

HAL_StatusTypeDef setAddress(uint8_t new_addr, uint32_t sensor_num)
{
	HAL_StatusTypeDef status = writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
	if (status != HAL_OK) {
		printf("[VL53] Failed to set address of sensor %lu to 0x%02X \r\n", (unsigned long)sensor_num, new_addr);
	};
  return status;
}

HAL_StatusTypeDef init()
{
  uint8_t model_id = readReg(IDENTIFICATION_MODEL_ID);
  if (model_id != 0xEE) { return HAL_ERROR;}

  writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
  writeReg(0x88, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  stop_variable = readReg(0x91);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);
  setSignalRateLimit(0.25);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  uint8_t ref_spad_map[6];
  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  writeReg(0xFF, 0x01);
  writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++) {
    if (i < first_spad_to_enable || spads_enabled == spad_count) {
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
      spads_enabled++;
    }
  }

  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0xFF, 0x00);
  writeReg(0x09, 0x00);
  writeReg(0x10, 0x00);
  writeReg(0x11, 0x00);
  writeReg(0x24, 0x01);
  writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00);
  writeReg(0xFF, 0x01);
  writeReg(0x4E, 0x2C);
  writeReg(0x48, 0x00);
  writeReg(0x30, 0x20);
  writeReg(0xFF, 0x00);
  writeReg(0x30, 0x09);
  writeReg(0x54, 0x00);
  writeReg(0x31, 0x04);
  writeReg(0x32, 0x03);
  writeReg(0x40, 0x83);
  writeReg(0x46, 0x25);
  writeReg(0x60, 0x00);
  writeReg(0x27, 0x00);
  writeReg(0x50, 0x06);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x96);
  writeReg(0x56, 0x08);
  writeReg(0x57, 0x30);
  writeReg(0x61, 0x00);
  writeReg(0x62, 0x00);
  writeReg(0x64, 0x00);
  writeReg(0x65, 0x00);
  writeReg(0x66, 0xA0);
  writeReg(0xFF, 0x01);
  writeReg(0x22, 0x32);
  writeReg(0x47, 0x14);
  writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00);
  writeReg(0xFF, 0x00);
  writeReg(0x7A, 0x0A);
  writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21);
  writeReg(0xFF, 0x01);
  writeReg(0x23, 0x34);
  writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF);
  writeReg(0x45, 0x26);
  writeReg(0x46, 0x05);
  writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06);
  writeReg(0x20, 0x1A);
  writeReg(0x43, 0x40);
  writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03);
  writeReg(0x35, 0x44);
  writeReg(0xFF, 0x01);
  writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09);
  writeReg(0x4C, 0x05);
  writeReg(0x4D, 0x04);
  writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00);
  writeReg(0x45, 0x20);
  writeReg(0x47, 0x08);
  writeReg(0x48, 0x28);
  writeReg(0x67, 0x00);
  writeReg(0x70, 0x04);
  writeReg(0x71, 0x01);
  writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00);
  writeReg(0x77, 0x00);
  writeReg(0xFF, 0x01);
  writeReg(0x0D, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0x01, 0xF8);
  writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  measurement_timing_budget_us = getMeasurementTimingBudget();
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
  setMeasurementTimingBudget(measurement_timing_budget_us);

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return HAL_ERROR; }

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return HAL_ERROR; }

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);
  return HAL_OK;
}

// ---> FIXED MULTI-SENSOR INITIALIZATION <---
HAL_StatusTypeDef vl53_init_multi() {
    HAL_StatusTypeDef status = HAL_OK;

    // Shut ALL sensors down first
    HAL_GPIO_WritePin(GPIOB, XSHUT_1_Pin|XSHUT_2_Pin|XSHUT_3_Pin|XSHUT_4_Pin|XSHUT_5_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // --- Init Sensor on XSHUT_4 ---
    HAL_GPIO_WritePin(GPIOB, XSHUT_4_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    vl53_set_target(VL53L0X_ADDR << 1); // Talk to default
    setAddress(VL53_ADDR_1, 4);         // Tell it to become 0x30
    vl53_set_target(VL53_ADDR_1_8BIT);  // Now our library talks to 0x30

    status = init();
    if (status == HAL_OK) {
        startContinuous(0);
        printf("[VL53] XSHUT4 Initialized at 0x%02X\r\n", VL53_ADDR_1);
    }

    // --- Init Sensor on XSHUT_5 ---
    HAL_GPIO_WritePin(GPIOB, XSHUT_5_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    vl53_set_target(VL53L0X_ADDR << 1); // The NEW sensor is at default
    setAddress(VL53_ADDR_2, 5);         // Tell it to become 0x31
    vl53_set_target(VL53_ADDR_2_8BIT);  // Now our library talks to 0x31

    status = init();
    if (status == HAL_OK) {
        startContinuous(0);
        printf("[VL53] XSHUT5 Initialized at 0x%02X\r\n", VL53_ADDR_2);
    }

    return status;
}

// ---> FIXED MULTI-SENSOR READ <---
HAL_StatusTypeDef vl53_read_multi(uint16_t *distance) {
	// Read Sensor 4
    vl53_set_target(VL53_ADDR_1_8BIT);
	distance[0] = readRangeContinuousMillimeters();

    // Read Sensor 5
    vl53_set_target(VL53_ADDR_2_8BIT);
	distance[1] = readRangeContinuousMillimeters();

	return HAL_OK;
}

// ---> ALL BELOW USE DYNAMIC ADDRESS (vl53_current_addr) <---

HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t value) {
    uint8_t write_buffer[2] = {reg, value};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, vl53_current_addr, write_buffer, 2, VL53_I2C_TIMEOUT_MS);
    return status;
}

HAL_StatusTypeDef writeReg16Bit(uint8_t reg, uint16_t value) {
    uint8_t write_buffer[3] = {reg, value >> 8, value};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, vl53_current_addr, write_buffer, 3, VL53_I2C_TIMEOUT_MS);
    return status;
}

HAL_StatusTypeDef writeReg32Bit(uint8_t reg, uint32_t value) {
    uint8_t write_buffer[5] = {reg, (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value)};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, vl53_current_addr, write_buffer, 5, VL53_I2C_TIMEOUT_MS);
    return status;
}

uint8_t readReg(uint8_t reg) {
  uint8_t value;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, vl53_current_addr, reg, 1, &value, 1, VL53_I2C_TIMEOUT_MS);
  if (status== HAL_OK) return value;
  return 0;
}

uint16_t readReg16Bit(uint8_t reg) {
    uint8_t buffer[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, vl53_current_addr, reg, 1, buffer, 2, VL53_I2C_TIMEOUT_MS);
    if (status == HAL_OK) return (buffer[1] | (buffer[0] << 8));
    return 0;
}

uint32_t readReg32Bit(uint8_t reg) {
    uint8_t buffer[4];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, vl53_current_addr, reg, 1, buffer, 4, VL53_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        uint32_t value = buffer[0] << 24;
        value |= buffer[1] << 16;
        value |= buffer[2] <<  8;
        value |= buffer[3];
        return value;
    }
    return 0;
}

HAL_StatusTypeDef writeMulti(uint8_t reg, uint8_t * src, uint8_t count) {
  return HAL_I2C_Mem_Write(hi2c, vl53_current_addr, reg, 1, src, count, VL53_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef readMulti(uint8_t reg, uint8_t * dst, uint8_t count) {
    return HAL_I2C_Mem_Read(hi2c, vl53_current_addr, reg, 1, dst, count, VL53_I2C_TIMEOUT_MS);
}

bool setSignalRateLimit(float limit_Mcps) {
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }
  writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

float getSignalRateLimit() {
  return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool setMeasurementTimingBudget(uint32_t budget_us) {
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) { used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead); }
  if (enables.dss) { used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead); }
  else if (enables.msrc) { used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead); }
  if (enables.pre_range) { used_budget_us += (timeouts.pre_range_us + PreRangeOverhead); }

  if (enables.final_range) {
    used_budget_us += FinalRangeOverhead;
    if (used_budget_us > budget_us) { return false; }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;
    uint32_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range) { final_range_timeout_mclks += timeouts.pre_range_mclks; }
    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
    measurement_timing_budget_us = budget_us;
  }
  return true;
}

uint32_t getMeasurementTimingBudget() {
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t budget_us = StartOverhead + EndOverhead;
  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) { budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead); }
  if (enables.dss) { budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead); }
  else if (enables.msrc) { budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead); }
  if (enables.pre_range) { budget_us += (timeouts.pre_range_us + PreRangeOverhead); }
  if (enables.final_range) { budget_us += (timeouts.final_range_us + FinalRangeOverhead); }

  measurement_timing_budget_us = budget_us;
  return budget_us;
}

bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (type == VcselPeriodPreRange) {
    switch (period_pclks) {
      case 12: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
      case 14: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
      case 16: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
      case 18: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
      default: return false;
    }
    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
    writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
    uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
    writeReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
  } else if (type == VcselPeriodFinalRange) {
    switch (period_pclks) {
      case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x30); writeReg(0xFF, 0x00); break;
      case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00); break;
      case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00); break;
      case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00); break;
      default: return false;
    }
    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
    if (enables.pre_range) { new_final_range_timeout_mclks += timeouts.pre_range_mclks; }
    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
  } else { return false; }

  setMeasurementTimingBudget(measurement_timing_budget_us);
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);
  return true;
}

uint8_t getVcselPulsePeriod(vcselPeriodType type) {
  if (type == VcselPeriodPreRange) return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  else if (type == VcselPeriodFinalRange) return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  else return 255;
}

void startContinuous(uint32_t period_ms) {
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_variable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  if (period_ms != 0) {
    uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);
    if (osc_calibrate_val != 0) { period_ms *= osc_calibrate_val; }
    writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    writeReg(SYSRANGE_START, 0x04);
  } else {
    writeReg(SYSRANGE_START, 0x02);
  }
}

void stopContinuous() {
  writeReg(SYSRANGE_START, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, 0x00);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
}

uint16_t readRangeContinuousMillimeters() {
  int32_t start_time = HAL_GetTick();
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (HAL_GetTick() - start_time > DEVICE_TIMEOUT) return 65535;
  }
  uint16_t range = readReg16Bit(RESULT_RANGE_STATUS + 10);
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

uint16_t readRangeSingleMillimeters() {
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_variable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(SYSRANGE_START, 0x01);

  int32_t start_time = HAL_GetTick();
  while (readReg(SYSRANGE_START) & 0x01) {
    if (HAL_GetTick() - start_time > DEVICE_TIMEOUT) return 65535;
  }
  return readRangeContinuousMillimeters();
}

bool getSpadInfo(uint8_t * count, bool * type_is_aperture) {
  uint8_t tmp;
  writeReg(0x80, 0x01); writeReg(0xFF, 0x01); writeReg(0x00, 0x00);
  writeReg(0xFF, 0x06); writeReg(0x83, readReg(0x83) | 0x04); writeReg(0xFF, 0x07); writeReg(0x81, 0x01);
  writeReg(0x80, 0x01); writeReg(0x94, 0x6b); writeReg(0x83, 0x00);

  int32_t start_time = HAL_GetTick();
  while (readReg(0x83) == 0x00) {
    if (HAL_GetTick() - start_time > DEVICE_TIMEOUT) { return false; }
    HAL_Delay(5);
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);
  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00); writeReg(0xFF, 0x06); writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01); writeReg(0x00, 0x01); writeReg(0xFF, 0x00); writeReg(0x80, 0x00);
  return true;
}

void getSequenceStepEnables(SequenceStepEnables * enables) {
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts) {
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);
  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->pre_range_mclks = decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);
  timeouts->final_range_mclks = decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range) { timeouts->final_range_mclks -= timeouts->pre_range_mclks; }
  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

uint16_t decodeTimeout(uint16_t reg_val) {
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t encodeTimeout(uint32_t timeout_mclks) {
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;
  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) { ls_byte >>= 1; ms_byte++; }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

bool performSingleRefCalibration(uint8_t vhv_init_byte) {
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte);
  int32_t start_time = HAL_GetTick();
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (HAL_GetTick() - start_time > DEVICE_TIMEOUT) { return false; }
    HAL_Delay(5);
  }
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(SYSRANGE_START, 0x00);
  return true;
}