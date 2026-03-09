% run_and_visualize.m
out = sim('PenduloRotativoSim', 'SimulationMode', 'normal');
send_to_visualizer(out.theta1_out, out.theta2_out);