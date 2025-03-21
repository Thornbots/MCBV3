namespace subsystems {

constexpr float C = 0.005;      // kg-s/m^2
constexpr float J = 0.028;      // kg-m^2
constexpr float UK = 0.03;      // N-m
constexpr float KB = 0.716;     // V-rad/s
constexpr float KT = 0.741;     // N-m/A
constexpr float RA = 8.705;     // ohm
constexpr float RATIO = 1;      // unitless
constexpr float VOLT_MAX = 24;  // V
                                // Position controller  constexprants
constexpr float KP = 11.3;      // 10.5;  // sec^-1

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

// calculated from existing  constexprants or robot independent (put after ifdef)

constexpr float KSTATIC = (UK * RA) / (KT * RATIO);  // A
constexpr float KV = KB * RATIO;                     // V-s/rad
constexpr float KA = J / (KT * RATIO);               // A-s^2/rad
constexpr float KVISC = C / (KT * RATIO);            // A-s/rad

constexpr float VELO_MAX = VOLT_MAX / (KB * RATIO);                // rad/s
constexpr float A_DECEL = 0.8 * VOLT_MAX * KT * RATIO / (J * RA);  // experimental per Alex_Y

}  // namespace subsystems