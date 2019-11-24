clear all
close all

gradient_threshold = 0.0017;
% load('heightmap1300.mat');
% total_length = size(height,2);
% side_length = round(sqrt(total_length));
% heightmap = reshape(height, [side_length, side_length]);

heightmap = csvread('height_highres.csv');
side_length = size(heightmap,1);
figure;
I = mat2gray(heightmap);
imshow(heightmap)

Gx = zeros(side_length, side_length);
Gy = zeros(side_length, side_length);

first_quadrant = heightmap(1:side_length/2,1:side_length/2);

[Gx1, Gy1] = imgradientxy(first_quadrant, 'intermediate');
Gx(1:side_length/2, 1:side_length/2) = Gx1;
Gy(1:side_length/2, 1:side_length/2) = Gy1;

second_quadrant = heightmap(1:side_length/2, side_length/2+1:end);
flipped_second_quadrant = flip(second_quadrant,2); 
[Gx2, Gy2] = imgradientxy(flipped_second_quadrant, 'intermediate');
Gx(1:side_length/2, side_length/2+1:end) = flip(Gx2,2);
Gy(1:side_length/2, side_length/2+1:end) = flip(Gy2,2);

third_quadrant = heightmap(side_length/2+1 :end, 1:side_length/2);
flipped_third_quadrant = flip(third_quadrant,1);
[Gx3, Gy3] = imgradientxy(flipped_third_quadrant, 'intermediate');
Gx(side_length/2+1:end, 1:side_length/2) = flip(Gx3,1);
Gy(side_length/2+1:end, 1:side_length/2) = flip(Gy3,1);

fourth_quadrant = heightmap(side_length/2+1:end, side_length/2+1:end);
flipped_fourth_quadrant = flip(fourth_quadrant,1);
flipped_fourth_quadrant = flip(flipped_fourth_quadrant,2);
[Gx4, Gy4] = imgradientxy(flipped_fourth_quadrant, 'intermediate');
flipped_Gx4 = flip(Gx4,1);
Gx(side_length/2+1:end, side_length/2+1:end) = flip(flipped_Gx4,2);
flipped_Gy4 = flip(Gy4,1);
Gy(side_length/2+1:end, side_length/2+1:end) = flip(flipped_Gy4,2);



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


global_occupancy_map = imresize(local_occupancy_map, 0.1);
figure;
imshow(global_occupancy_map)
imwrite(global_occupancy_map, 'global_occupancy_map.png')

webots_map = imresize(heightmap, 0.25);
webots_map = reshape(webots_map, 1, 650*650);
csvwrite('webots_map.csv', webots_map); 
