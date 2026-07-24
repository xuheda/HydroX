#pragma once
/**
 * energy_model.h — Battery energy model
 *
 * Features:
 *   - Accumulates motor electric power + hotel load (sensors/computer/actuators)
 *   - Tracks consumed energy (Wh) and remaining SOC
 *   - Internal resistance voltage drop -> actual terminal voltage
 *   - Estimates remaining runtime (based on the average power of the last 10 seconds)
 *
 * Parameter reference: Typical torpedo AUV (24V / 200Wh LiFePO4)
 *   capacity_Wh = 200.0   (8.33 Ah @24V)
 *   V_nom       = 24.0 V
 *   P_hotel     = 15.0 W  (DVL+IMU+autopilot board+lights)
 *   R_int       = 0.05 Ohm (battery internal resistance)
 *
 * Usage (within the 100Hz loop in main_sitl.cpp):
 *   EnergyState es = energy.update(motor_state.power_W, dt);
 */
#include <cmath>
#include <deque>

namespace hydrox
{

    struct EnergyState
    {
        double power_motor_W = 0.0;   // Motor instantaneous electrical power (W)
        double power_hotel_W = 0.0;   // Hotel load power (W, constant)
        double power_total_W = 0.0;   // Total power (W)
        double current_total_A = 0.0; // Total current (A)
        double V_terminal = 0.0;      // Battery terminal voltage (V, including internal resistance voltage drop)
        double energy_Wh = 0.0;       // Consumed energy (Wh)
        double soc = 1.0;             // Remaining SOC [0, 1]
        double runtime_rem_s = 0.0;   // Estimated remaining runtime (s), -1 = unable to estimate
    };

    class EnergyModel
    {
    public:
        struct Params
        {
            double capacity_Wh = 500000.0; // Deployment/SITL validation capacity (Wh)
            double V_nom = 24.0;        // Nominal voltage (V)
            double P_hotel = 15.0;      // Hotel load (W)
            double R_int = 0.05;        // Battery internal resistance (Ohm)
        };

        explicit EnergyModel(const Params &p = {}) : _p(p)
        {
            _state.V_terminal = p.V_nom;
            _state.power_hotel_W = p.P_hotel;
            _state.runtime_rem_s = (p.capacity_Wh * 3600.0) / p.P_hotel;
        }

        void reset()
        {
            _energy_Wh = 0.0;
            _power_history.clear();
            _state = EnergyState{};
            _state.V_terminal = _p.V_nom;
            _state.power_hotel_W = _p.P_hotel;
        }

        /**
         * update — Called once per GNC tick
         * @param motor_power_W  Motor electrical power of this tick (W, from MotorState)
         * @param dt             Time step size (s)
         * @return               Updated energy state
         */
        EnergyState update(double motor_power_W, double dt)
        {
            const double p_total = motor_power_W + _p.P_hotel;
            const double i_total = (_p.V_nom > 1e-6) ? p_total / _p.V_nom : 0.0;
            const double v_term = _p.V_nom - i_total * _p.R_int;

            // Integrate energy (Wh)
            _energy_Wh += p_total * dt / 3600.0;
            _energy_Wh = std::min(_energy_Wh, _p.capacity_Wh); // Do not exceed capacity

            // SOC
            const double soc = 1.0 - _energy_Wh / _p.capacity_Wh;

            // Moving window average power (recent 10s, @100Hz = 1000 samples)
            _power_history.push_back(p_total);
            if (_power_history.size() > 1000)
                _power_history.pop_front();
            double p_avg = 0.0;
            for (double pw : _power_history)
                p_avg += pw;
            p_avg /= static_cast<double>(_power_history.size());

            // Remaining runtime (s)
            const double energy_rem_Wh = _p.capacity_Wh * soc;
            const double runtime_s = (p_avg > 0.5)
                                         ? (energy_rem_Wh / p_avg) * 3600.0
                                         : -1.0; // Power too small, unable to estimate

            _state.power_motor_W = motor_power_W;
            _state.power_hotel_W = _p.P_hotel;
            _state.power_total_W = p_total;
            _state.current_total_A = i_total;
            _state.V_terminal = v_term;
            _state.energy_Wh = _energy_Wh;
            _state.soc = soc;
            _state.runtime_rem_s = runtime_s;

            return _state;
        }

        const EnergyState &state() const { return _state; }

    private:
        Params _p;
        double _energy_Wh = 0.0;
        EnergyState _state;
        std::deque<double> _power_history; // Sliding window (recent 10s)
    };

} // namespace hydrox
