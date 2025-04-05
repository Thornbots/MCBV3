constexpr float VOLT_MAX = 24;     // V
constexpr float CURRENT_MAX = 10;  // A
constexpr float lat = .008;
constexpr float DT = .002;
constexpr int Q_SIZE = lat / DT;  // make this fancy later

#if defined(HERO)
constexpr float C = 0.005;                                         // kg-s/m^2
constexpr float J = 0.028;                                         // kg-m^2
constexpr float UK = 0.03;                                         // N-m
constexpr float KB = 0.716;                                        // V-rad/s
constexpr float KT = 0.741;                                        // N-m/A
constexpr float RA = 8.705;                                        // ohm
constexpr float RATIO = 1;                                         // unitless
                                                                   // Position controller  constexprants
constexpr float KP = 11.3;                                         // 10.5;  // sec^-1
constexpr float THETA_DOT_BREAK = 0.1;                             // rad/s
constexpr float A_DECEL = 0.7 * VOLT_MAX * KT * RATIO / (J * RA);  // experimental per Alex_Y

// Feedforward  constexprants
constexpr float A_SCALE = 0.9;  // 0.8            // unitless

// Gain scheduling
constexpr float KDT = -0.47;     // unitless
constexpr float KDT_REV = -0.7;  // unitless

// Velocity feedback
constexpr float KPV = 0.7;                      // A-s/rad
constexpr float KIV = 30;                       // A/rad
constexpr float IV_MAX = 2 / KIV;               // units TBD
constexpr float INT_THRESH = VOLT_MAX * 0.85l;  // V
constexpr float TAKEBACK = 0.01;                // unitless

#elif defined(SENTRY)
constexpr float C = 0.12;                                          // kg-s/m^2
constexpr float J = 0.04;//259;                                        // kg-m^2
constexpr float UK = 0;                                          // N-m
constexpr float KB = 0.741;                                       // V-rad/s
constexpr float KT = 0.716;                                       // N-m/A
constexpr float RA = 8.705;                                       // ohm
constexpr float RATIO = 1.0f;                                         // unitless
                                                                   // Position controller  constexprants
constexpr float KP = 20;                                            // 10.5;  // sec^-1
constexpr float THETA_DOT_BREAK = 10;                              // rad/s
constexpr float A_DECEL = 0.2 * VOLT_MAX * KT * RATIO / (J * RA);  // experimental per Alex_Y

// Feedforward  constexprants
constexpr float A_SCALE = 0.9;  // 0.8            // unitless

// Gain scheduling
constexpr float KDT = -0.2;      // unitless
constexpr float KDT_REV = -0.2;  // unitless

// Velocity feedback
constexpr float KPV = 2.8;                       // A-s/rad
constexpr float KIV = 0.02;                      // A/rad
constexpr float IV_MAX = 0.1 / KIV;              // units TBD
constexpr float INT_THRESH = VOLT_MAX * 0.85;  // V
constexpr float TAKEBACK = 0.1;               // unitless
#elif defined(INFANTRY)

constexpr float C = 0.12;  // kg-s/m^2
constexpr float J = 0.03;  // kg-m^2
constexpr float UK = 0.0;  // N-m

constexpr float KB = 0.4538;                                       // V-rad/s
constexpr float KT = 0.4414;                                       // N-m/A
constexpr float RA = .5592;                                        // ohm
constexpr float RATIO = 54.0 / 27.0;                               // unitless
                                                                   // Position controller  constexprants
constexpr float KP = 15;                                           // 10.5;  // sec^-1
constexpr float THETA_DOT_BREAK = 10;                              // rad/s
constexpr float A_DECEL = 0.3 * VOLT_MAX * KT * RATIO / (J * RA);  // experimental per Alex_Y

// Feedforward  constexprants
constexpr float A_SCALE = 0.9;  // 0.8            // unitless

// Gain scheduling
constexpr float KDT = -0.1;      // unitless
constexpr float KDT_REV = -0.1;  // unitless

// Velocity feedback
constexpr float KPV = 3.5;         // A-s/rad
constexpr float KIV = .1;          // A/rad
constexpr float IV_MAX = 3 / KIV;  //.2 / KIV;              // units TBD

constexpr float INT_THRESH = VOLT_MAX * 0.85;  // V
constexpr float TAKEBACK = 0.1;
#else
constexpr float C = 0.005;                                         // kg-s/m^2
constexpr float J = 0.028;                                         // kg-m^2
constexpr float UK = 0.03;                                         // N-m
constexpr float KB = 0.716;                                        // V-rad/s
constexpr float KT = 0.741;                                        // N-m/A
constexpr float RA = 8.705;                                        // ohm
constexpr float RATIO = 1;                                         // unitless
                                                                   // Position controller  constexprants
constexpr float KP = 11.3;                                         // 10.5;  // sec^-1
constexpr float THETA_DOT_BREAK = 999;                             // rad/s
constexpr float A_DECEL = 0.7 * VOLT_MAX * KT * RATIO / (J * RA);  // experimental per Alex_Y

// Feedforward  constexprants
constexpr float A_SCALE = 0.9;  // 0.8            // unitless

// Gain scheduling
constexpr float KDT = -0.47;     // unitless
constexpr float KDT_REV = -0.7;  // unitless

// Velocity feedback
constexpr float KPV = 0.7;                     // A-s/rad
constexpr float KIV = 30;                      // A/rad
constexpr float IV_MAX = 2 / KIV;              // units TBD
constexpr float INT_THRESH = VOLT_MAX * 0.85;  // V
constexpr float TAKEBACK = 0.01;               // unitless
#endif

// calculated from existing  constexprants or robot independent (put after ifdef)

constexpr float KSTATIC = (UK * RA) / (KT * RATIO);  // A
constexpr float KV = KB * RATIO;                     // V-s/rad
constexpr float KA = J / (KT * RATIO);               // A-s^2/rad
constexpr float KVISC = C / (KT * RATIO);            // A-s/rad

constexpr float VELO_MAX = VOLT_MAX / (KB * RATIO);  // rad/s