clear all
close all

gradient_threshold = 0.0017;
% load('heightmap1300.mat');
% total_length = size(height,2);
% side_length = round(sqrt(total_length));
% heightmap = reshape(height, [side_length, side_length]);

% heightmap = csvread('height_highres_big_pit_pit_centre.csv');

heightmap = zeros(8,8);
heightmap(4:5,4:5) = [-80,-80;-80,-80];
side_length = size(heightmap,1);
figure;
I = mat2gray(heightmap);
imshow(heightmap)

Gx = zeros(side_length, side_length);
Gy = zeros(side_length, side_length);

first_quadrant = heightmap(1:side_length/2,1:side_length/2);
flipped_first_quadrant = flip(first_quadrant, 1);
flipped_first_quadrant = flip(flipped_first_quadrant,2);
[Gx1, Gy1] = imgradientxy(flipped_first_quadrant, 'intermediate');
flipped_Gx1 = flip(Gx1,1);
Gx(1:side_length/2, 1:side_length/2) = flip(flipped_Gx1,2);
flipped_Gy1 = flip(Gy1,1);
Gy(1:side_length/2, 1:side_length/2) = flip(flipped_Gy1,2);

second_quadrant = heightmap(1:side_length/2, side_length/2+1:end);
flipped_second_quadrant = flip(second_quadrant,1); 
[Gx2, Gy2] = imgradientxy(flipped_second_quadrant, 'intermediate');
Gx(1:side_length/2, side_length/2+1:end) = flip(Gx2,1);
Gy(1:side_length/2, side_length/2+1:end) = flip(Gy2,1);

third_quadrant = heightmap(side_length/2+1 :end, 1:side_length/2);
flipped_third_quadrant = flip(third_quadrant,2);
[Gx3, Gy3] = imgradientxy(flipped_third_quadrant, 'intermediate');
Gx(side_length/2+1:end, 1:side_length/2) = flip(Gx3,2);
Gy(side_length/2+1:end, 1:side_length/2) = flip(Gy3,2);

fourth_quadrant = heightmap(side_length/2+1:end, side_length/2+1:end);
[Gx4, Gy4] = imgradientxy(fourth_quadrant, 'intermediate');
Gx(side_length/2+1:end, side_length/2+1:end) = Gx4;
Gy(side_length/2+1:end, side_length/2+1:end) = Gy4;


[Gmag, Gdir] = imgradient(Gx,Gy);
Gmag = mat2gray(Gmag);
figure
imshow(Gmag);
BM = (Gmag >= 0 & Gmag < gradient_threshold);
pit_mask = heightmap == -80;
local_occupancy_map = ~pit_mask & BM;
figure;
imshow(local_occupancy_map)
imwrite(local_occupancy_map, 'local_occupancy_map.png')
csvwrite('occupancy_local_map.csv', local_occupancy_map);

global_occupancy_map = imresize(local_occupancy_map, 0.1);
global_heightmap = imresize(heightmap, 0.1);
figure;
imshow(global_occupancy_map)
img_global_heightmap = mat2gray(global_heightmap);
imwrite(img_global_heightmap, 'global_map.png')
csvwrite('elevation_global_map.csv', global_heightmap);
csvwrite('occupancy_global_map.csv', global_occupancy_map);
imwrite(global_occupancy_map, 'global_occupancy_map.png')

webots_map = imresize(heightmap, 0.25);
webots_map = reshape(webots_map, 1, 650*650);
csvwrite('webots_map.csv', webots_map); 
