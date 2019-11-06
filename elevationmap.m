clear all
close all
% load('heightmap.mat')
% side = size(heightmap,2);
% length = round(sqrt(side));
% heightmap = reshape(heightmap, [length, length]);


% heightmap = csvread('sldem_snippet_with_rocks.csv');
% I = mat2gray(heightmap);
% figure
% imshow(I)
% [r, c] = size(heightmap);
% 
% flatten_heightmap = reshape(heightmap, [1, r*c]);
% csvwrite('heightmap.csv',flatten_heightmap);

% t = Tiff('worlds/NAC_DTM_TSILKVSKIY2_CLRSHADE_20170911_164042.tif', 'r');
% imageData = read(t);
% [r, c] = size(imageData);
% flatten_heightmap = reshape(imageData, [1, r*c]);
% csvwrite('heightmap.csv',flatten_heightmap);

 localmap = csvread("globalmap.csv");
 I = mat2gray(localmap);
 Igrad = imgradient(I)
 BM = (Igrad >= 0 & Igrad < 0.09);
 imshow(BM)
 
 
 
 

