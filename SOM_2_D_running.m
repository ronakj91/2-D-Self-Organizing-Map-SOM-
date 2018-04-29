%% SOM 2-D
close all
clc
samples = 200;
som_row = 10;
som_col = 10;
eta_i = 0.1;
sigma_i = 1.5;    %initial sigma
tau1 = 3000;
tau2 = 80;
h = 0;          %neighbourhood function

%% Generation of data for input space
input_space = [rand(2,samples/2)-1 rand(2,samples/2)];
i = randperm(samples);
input_space = input_space(:,i);

%input_space = rand(2,samples);



%% Initialization of SOM
weights = 0.8*rand(som_row,som_col,2)-1;


%% SOM Algo

plot(input_space(1,:),input_space(2,:),'.');
hold on
plot(weights(:,:,1),weights(:,:,2),'bo');
hold off
pause(0.1)

for m = 1:300
    eta = eta_i*exp(-(m/tau2));
    display(m)
    display(eta)
    for i = 1:samples
        
        % Finding out winning neuron
        dist = zeros(som_row,som_col);
        for row = 1:som_row
            for col = 1:som_col
                w = [weights(row,col,1);weights(row,col,2)];
                dist(row,col) = norm(w-input_space(:,i));
            end
        end
        [M,I] = min(dist);
        [M,in] = min(M);
        I = [I(in) in];
        index_min = I';
        
        %weight updation
        sigma = sigma_i*exp(-(i/tau1));
        for row = 1:som_row
            for col = 1:som_col
                eucl_dist = norm([row;col]-index_min);
                h = exp(-(eucl_dist^2/(2*sigma^2)));
                weights(row,col,1) = weights(row,col,1) + eta*h*(input_space(1,i)-weights(row,col,1));
                weights(row,col,2) = weights(row,col,2) + eta*h*(input_space(2,i)-weights(row,col,2));
                
            end
        end
        
        
    end
    plot(input_space(1,:),input_space(2,:),'.');
    hold on
    plot(weights(:,:,1),weights(:,:,2),'bo');
    hold off
    pause(0.1)
end

%save('SOM_eg_2_8_2_D');



