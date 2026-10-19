%% Integrators.m

% Assisted by Google Gemini 3.1 Pro

classdef Integrators
    methods (Static)
        
        function state_next = euler(odefun, t, state, dt)
            % EULER Standard first-order forward Euler integration
            state_dot  = odefun(t, state);
            state_next = state + state_dot * dt;
        end

        function state_next = rk4(odefun, t, state, dt)
            % RK4 Standard fourth-order Runge-Kutta integration
            k1 = odefun(t, state);
            k2 = odefun(t + dt/2, state + k1 * (dt/2));
            k3 = odefun(t + dt/2, state + k2 * (dt/2));
            k4 = odefun(t + dt,   state + k3 * dt);
            
            state_next = state + (k1 + 2*k2 + 2*k3 + k4) * (dt/6);
        end
        
    end
end