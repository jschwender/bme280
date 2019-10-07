// control register 0xF4 
  const char      temp_oversampling_1 = 0b00100000;
  const char      temp_oversampling_2 = 0b01000000;
  const char      temp_oversampling_4 = 0b01100000;
  const char      temp_oversampling_8 = 0b10000000;
  const char     temp_oversampling_16 = 0b10100000;

  const char  pressure_oversampling_1 = 0b00000100;
  const char  pressure_oversampling_2 = 0b00001000;
  const char  pressure_oversampling_4 = 0b00001100;
  const char  pressure_oversampling_8 = 0b00010000;
  const char pressure_oversampling_16 = 0b00010100;

  const char   mode_sleep = 0b00000000;
  const char  mode_forced = 0b00000010;
  const char  mode_normal = 0b00000011;
// config register 0xF5
  const char     ts_500u = 0b00000000;
  const char   ts_62500u = 0b00100000;
  const char     ts_125m = 0b01000000;
  const char     ts_250m = 0b01100000;
  const char     ts_500m = 0b10000000;
  const char        ts_1 = 0b10100000;
  const char      ts_10m = 0b11000000;
  const char      ts_20m = 0b11100000;

  const char    filter_off = 0b00000000;
  const char      filter_2 = 0b00000100;
  const char      filter_4 = 0b00001000;
  const char      filter_8 = 0b00001100;
  const char     filter_16 = 0b00010000;

  const char  spi_enable = 0b00000001;
// humid control register values
  const char   hum_oversampling_o = 0b00000000;
  const char   hum_oversampling_1 = 0b00000001;
  const char   hum_oversampling_2 = 0b00000010;
  const char   hum_oversampling_4 = 0b00000011;
  const char   hum_oversampling_8 = 0b00000100;
  const char  hum_oversampling_16 = 0b00000101;
