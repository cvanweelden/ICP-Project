PATH = '../../../results/parameter_tuning/';

datasets = {'xyz', 'desk'};
methods{1}.name = 'fpfh';
methods{1}.parameters = 0.1:0.05:0.35;
methods{2}.name = 'icp';
methods{2}.parameters = 0.05:0.05:0.25;

for m = methods
    
    mean_error_q = zeros(length(datasets), length(m{1}.parameters));
    mean_error_t = zeros(length(datasets), length(m{1}.parameters));
    
    for id = 1:length(datasets)
        d = datasets{id};
        
        for i = 1:length(m{1}.parameters)
            filename = sprintf('%s_%s_%.2f.txt', d, m{1}.name, m{1}.parameters(i));
            filename = fullfile(PATH, filename);
            results = parse_eval_file(filename);
            
            mean_error_q(id, i) = mean(results.error_q);
            mean_error_t(id, i) = mean(results.error_t);
        end
        
    end
    
    figure;
    hold all;
    title(sprintf('Rotation error for %s', m{1}.name));
    for id = 1:length(datasets)
        plot(m{1}.parameters, mean_error_q(id,:));
    end
    legend(datasets{:});

    figure;
    hold all;
    title(sprintf('Translation error for %s', m{1}.name));
    for id = 1:length(datasets)
        plot(m{1}.parameters, mean_error_t(id,:));
    end
    legend(datasets{:});
    
end