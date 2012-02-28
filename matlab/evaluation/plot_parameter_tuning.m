PATH = '../../../results/parameter_tuning/';

datasets = {'xyz', 'desk'};
methods{1}.name = 'fpfh';
methods{1}.parameters = 0.1:0.05:0.3;
methods{2}.name = 'icp';
methods{2}.parameters = 0.05:0.05:0.25;

for d = datasets
    for m = methods
        
        mean_error_q = zeros(1, length(m{1}.parameters));
        mean_error_t = zeros(1, length(m{1}.parameters));
        
        for i = 1:length(m{1}.parameters)
            filename = sprintf('%s_%s_%.2f.txt', d{1}, m{1}.name, m{1}.parameters(i));
            filename = fullfile(PATH, filename);
            results = parse_eval_file(filename);
            
            mean_error_q(i) = mean(results.error_q);
            mean_error_t(i) = mean(results.error_t);
        end
        
        figure;
        hold all;
        title(sprintf('Error for %s on %s', m{1}.name, d{1}));
        plot(m{1}.parameters, mean_error_q);
        %plot(m{1}.parameters, mean_error_t);
        legend('Rotation error (radian)', 'Translation error (meters)');
        
    end
end