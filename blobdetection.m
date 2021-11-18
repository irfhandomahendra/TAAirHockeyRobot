clear all;
delete(instrfindall);
arduino = serial('COM5','BaudRate',115200,'DataBits',8,'StopBits',1,'Parity','none');
fopen(arduino);
pause(2);
greenThresh = 0.03;
blueThresh = 0.062;     
vidDevice = imaq.VideoDevice('winvideo', 2, 'RGB24_320x240', ... 
                    'ROI', [1 35 320 200], ...
                    'ReturnedColorSpace', 'rgb');
vidInfo = imaqhwinfo(vidDevice); 
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 100, ...
                                'MaximumBlobArea', 2000, ...
                                'MaximumCount', 1);
nFrame = 0; 
logdata = [];
tic;
while(nFrame <= 1000)
    rgbFrame = step(vidDevice); 
    diffFrameGreen = imsubtract(rgbFrame(:,:,2), rgb2gray(rgbFrame)); 
    diffFrameGreen = medfilt2(diffFrameGreen, [3 3]); 
    binFrameGreen = im2bw(diffFrameGreen, greenThresh); 
    diffFrameBlue = imsubtract(rgbFrame(:,:,3), rgb2gray(rgbFrame)); 
    diffFrameBlue = medfilt2(diffFrameBlue, [3 3]); 
    binFrameBlue = im2bw(diffFrameBlue, blueThresh); 
    [centroidGreen, bboxGreen] = step(hblob, binFrameGreen); 
    centroidGreen = uint16(centroidGreen); 
    [centroidBlue, bboxBlue] = step(hblob, binFrameBlue); 
    centroidBlue = uint16(centroidBlue); 
    frameTimestamp = toc;
    timestamp = frameTimestamp;
    marker = 1000;
    parameters = [centroidGreen centroidBlue]
    logdata = [logdata; parameters];
    if nFrame > 0
    fprintf(arduino,'%d/n', double(marker));
    fprintf(arduino,'%d/n', double(marker));
    fprintf(arduino,'%d/n', double(parameters(:)));
    end
    FPS = nFrame/timestamp
    nFrame = nFrame+1;
end
release(vidDevice);
fclose(arduino);