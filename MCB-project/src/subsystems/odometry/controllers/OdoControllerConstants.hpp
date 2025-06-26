namespace odo{

constexpr float VOLT_MAX = 24;     // V
constexpr float lat = .008;
constexpr float DT = .002;
constexpr int Q_SIZE = lat / DT;  // make this fancy later

constexpr float C = 0.000;                                          // kg-s/m^2
constexpr float J = 0.001;//259;                                        // kg-m^2
constexpr float UK = -0.01;                                          // N-m
constexpr float KB = 0.716;                                       // V-rad/s
constexpr float KT = 0.741;                                       // N-m/A
constexpr float RA = 8.705;                                       // ohm
constexpr float RATIO = 1;                                         // unitless
                                                                   // Position controller  constexprants
constexpr float KP = 50;                                            // 10.5;  // sec^-1
constexpr float THETA_DOT_BREAK = 6;                              // rad/s
constexpr float A_DECEL = 0.15 * VOLT_MAX * KT * RATIO / (J * RA);  // experimental per Alex_Y

// Feedforward  constexprants
constexpr float A_SCALE = 0.9;  // 0.8            // unitless

// Gain scheduling
constexpr float KDT = 0;      // unitless
constexpr float KDT_REV = 0;  // unitless

// Velocity feedback
constexpr float KPV = .15; //%0.04;                       // A-s/rad
constexpr float KIV = 1;                      // A/rad
constexpr float IV_MAX = 0/ KIV;              // units TBD
constexpr float INT_THRESH = VOLT_MAX * 0.85;  // V
constexpr float TAKEBACK = 0.1;               // unitless
constexpr float CURRENT_MAX = 5;  // A


// calculated from existing  constexprants or robot independent (put after ifdef)

constexpr float KSTATIC = (UK * RA) / (KT * RATIO);  // A
constexpr float KV = KB * RATIO;                     // V-s/rad
constexpr float KA = J / (KT * RATIO);               // A-s^2/rad
constexpr float KVISC = C / (KT * RATIO);            // A-s/rad

constexpr float VELO_MAX = VOLT_MAX / (KB * RATIO);  // rad/s

}