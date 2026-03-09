function send_to_visualizer(theta1_ts, theta2_ts, port)
% SEND_TO_VISUALIZER  Sends simulation results to the Python visualizer over TCP.
%
% Accepts Timeseries objects directly from To Workspace blocks and
% resamples them onto a common uniform time grid (handles variable-step solvers).
%
% StopFcn callback (with Single simulation output ON):
%   send_to_visualizer(out.theta1_out, out.theta2_out)
%
% StopFcn callback (with Single simulation output OFF):
%   send_to_visualizer(theta1_out, theta2_out)
%
% Manual usage:
%   out = sim('your_model');
%   send_to_visualizer(out.theta1_out, out.theta2_out);
%
% Arguments:
%   theta1_ts - Timeseries of pendulum angle (rad)
%   theta2_ts - Timeseries of wheel angle (rad)
%   port      - (optional) TCP port, default 25000

    if nargin < 3
        port = 25000;
    end

    % Extract time and data vectors from whatever format MATLAB gives us.
    % Supports: timeseries, timetable, struct with .Time/.Data, or raw arrays.
    [t1, d1] = extract_signal(theta1_ts);
    [t2, d2] = extract_signal(theta2_ts);

    % Debug: show what we got
    fprintf('[VIZ] theta1: %d points, t=[%.4f, %.4f]\n', length(t1), t1(1), t1(end));
    fprintf('[VIZ] theta2: %d points, t=[%.4f, %.4f]\n', length(t2), t2(1), t2(end));

    % Build a common uniform time vector at 1 kHz
    t_start = max(t1(1), t2(1));
    t_end   = min(t1(end), t2(end));
    dt = 0.001;
    time_data = (t_start : dt : t_end)';

    % Interpolate both signals onto the common time grid
    theta1_data = interp1(t1, d1, time_data, 'linear', 'extrap');
    theta2_data = interp1(t2, d2, time_data, 'linear', 'extrap');

    N = length(time_data);

    fprintf('[VIZ] Sending %d samples to visualizer on port %d...\n', N, port);

    % Pack data: interleaved [t0, th1_0, th2_0, t1, th1_1, th2_1, ...]
    interleaved = zeros(N, 3);
    interleaved(:, 1) = time_data;
    interleaved(:, 2) = theta1_data;
    interleaved(:, 3) = theta2_data;
    payload = typecast(reshape(interleaved', 1, []), 'uint8');  % flatten row-major to bytes

    % Connect and send
    try
        t = tcpclient('127.0.0.1', port, 'Timeout', 5);
        
        % Send N as int32 (4 bytes)
        write(t, typecast(int32(N), 'uint8'));
        
        % Send payload in chunks (tcpclient can handle large writes,
        % but chunking is safer for very long simulations)
        chunk_size = 65536;
        offset = 1;
        total = length(payload);
        while offset <= total
            end_idx = min(offset + chunk_size - 1, total);
            write(t, payload(offset:end_idx));
            offset = end_idx + 1;
        end
        
        clear t;  % close connection
        fprintf('[VIZ] Done! %d samples sent (%.2f s of simulation).\n', N, time_data(end));
        
    catch err
        fprintf('[VIZ] Error: %s\n', err.message);
        fprintf('[VIZ] Is the Python visualizer running? Start it first with:\n');
        fprintf('[VIZ]   python pendulum_viz.py\n');
    end
end


function [t, d] = extract_signal(sig)
% EXTRACT_SIGNAL  Pulls time and data vectors from any Simulink output format.
    if isa(sig, 'timeseries')
        t = sig.Time(:);
        d = sig.Data(:);
    elseif isa(sig, 'timetable')
        t = seconds(sig.Time);
        t = t(:);
        d = sig{:,1};
        d = d(:);
    elseif isstruct(sig) && isfield(sig, 'Time') && isfield(sig, 'Data')
        t = sig.Time(:);
        d = sig.Data(:);
    elseif isstruct(sig) && isfield(sig, 'time') && isfield(sig, 'signals')
        t = sig.time(:);
        d = sig.signals.values(:);
    else
        % Last resort: print what we actually got so user can report back
        fprintf('[VIZ] Unknown signal type: %s\n', class(sig));
        fprintf('[VIZ] Fields/properties:\n');
        disp(fieldnames(sig));
        error('Cannot extract time/data from input of class "%s". See above.', class(sig));
    end
end