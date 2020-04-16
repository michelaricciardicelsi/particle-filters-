function likelihood = exampleHelperRobotMeasurementLikelihood(pf, predictParticles, measurement)
    %{
    prob = 1.0;
    for i in range(len(landmarks)):
        dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
        prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
    return prob
    %}
    
    % Y = normpdf(X,MU,SIGMA)
    sensor_noise = 3;
    global robot
    predictMeasurement = zeros(pf.NumParticles, robot.LaserSensor.NumReadings);
    likelihood = zeros(pf.NumParticles, 1);
    measurement(isnan(measurement)) = robot.LaserSensor.MaxRange;
    for i = 1:pf.NumParticles
        tempMeasurement = robot.LaserSensor.getReading(predictParticles(i,:));
        tempMeasurement(isnan(tempMeasurement)) = robot.LaserSensor.MaxRange;
        predictMeasurement(i,:) = tempMeasurement;
        
        prob = 1.0;
        for j = 1:robot.LaserSensor.NumReadings
            % prob = prob*gaussian(measurement(j),sensor_noise,predictMeasurement(i,j));
            prob = prob*normpdf(predictMeasurement(i,j),measurement(j),sensor_noise);
        end
        likelihood(i) = prob;
    end
    
    %{
    % The measurement contains all state variables
    predictMeasurement = predictParticles;
    % Calculate observed error between predicted and actual measurement
    % NOTE in this example, we don't have full state observation, but only
    % the measurement of current pose, therefore the measurementErrorNorm
    % is only based on the pose error.
    measurementError = bsxfun(@minus, predictMeasurement(:,1:3), measurement);
    measurementErrorNorm = sqrt(sum(measurementError.^2, 2));
    % Normal-distributed noise of measurement
    % Assuming measurements on all three pose components have the same error distribution
    measurementNoise = eye(3);
    % Convert error norms into likelihood measure.
    % Evaluate the PDF of the multivariate normal distribution
    likelihood = 1/sqrt((2*pi).^3 * det(measurementNoise)) * exp(-0.5 * measurementErrorNorm);
    %}
end
