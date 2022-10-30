% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction.
function myMap = occGridMapping(ranges, scanAngles, pose, param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 

% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);

% num_scans = size(scanAngles,1);
% 
% for j = 1:N % for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)
%     % as per the instruction in the pdf
%     x = pose(1,j);
%     y = pose(2,j);
%     theta = pose(3,j);
%     
%     alpha = scanAngles; 
%     d = ranges(:,j); 
%     
%     real_loc = [d.*cos(theta+alpha), -d.*sin(theta+alpha)]' + repmat([x;y],1,num_scans);
%     % Find occupied-measurement cells and free-measurement cells
%     
%     grid_index =  ceil(myResol*real_loc) + repmat(myorigin,1,num_scans);
%     
%     pos_curr = ceil(myResol*[pose(1,j);pose(2,j)]) + myorigin;
%     
%     occ_indices = sub2ind(size(myMap), grid_index(2,:), grid_index(1,:)); 
%     
%     myMap(occ_indices) = myMap(occ_indices) + lo_occ; % Update the occ information of the map
%     
%     % Update the log-odds
%     
%     for i = 1:num_scans
%         
%         [freex, freey] = bresenham(pos_curr(1),pos_curr(2),grid_index(1,i),grid_index(2,i));
% 
%         free_indices = sub2ind(size(myMap), freey, freex);
%         
%         myMap(free_indices) = myMap(free_indices) - lo_free; % Update the free information of the map
%         
%     end

r = param.resol;
N = size(pose,2);
n = size(scanAngles,1);
for j = 1:N % for each time,
    for angle = 1:n
      % Find grids hit by the rays (in the gird map coordinate)
      x_o = ranges(angle,j) * cos(scanAngles(angle,1) + pose(3,j)) + pose(1,j);
      y_o = -1*ranges(angle,j) * sin(scanAngles(angle,1) + pose(3,j)) + pose(2,j);
     
      % Find occupied-measurement cells and free-measurement cells
      occ= [ceil(x_o*r)+myorigin(1)  ceil(y_o*r) + myorigin(2)];
      car = [ceil(pose(1,j)*r) + myorigin(1)  ceil(pose(2,j)*r) + myorigin(2)];
      if occ(2)>0 && occ(1)>0 &&  occ(2) < param.size(1)+1 &&  occ(1) < param.size(2)+1
        myMap(occ(2),occ(1)) =  myMap(occ(2),occ(1))  + lo_occ;
      end
      [freex,freey]  = bresenham(car(1),car(2),occ(1),occ(2));  
      if size(freex,2)>0
        freex_ = freex';
        freey_ = freey';
        %disp(size(freex_))
        del_index = freex_<1 | freex_> param.size(2) | freey_<1 | freey_>param.size(1);
        %disp(size(del_index))
        freex_(del_index) = [];
        freey_(del_index) = [];
        freex = freex_';
        freey = freey_';
        free = sub2ind(size(myMap),freey,freex);
        myMap(free) = myMap(free)-lo_free;
      
      end
    end
    myMap = min(myMap,lo_max);
    myMap = max(myMap,lo_min);
end
% myMap(myMap<lo_min) = lo_min; % Prevent the map from becoming too certain
% myMap(myMap>lo_max) = lo_max;


%visualizing the map
figure(1),
imagesc(myMap); hold on;
plot(pose(1,1),pose(1,2),'rx','LineWidth',3); % indicate start point
axis equal;
end

