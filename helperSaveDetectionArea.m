% Inputs to the function are axes handle to the figure containing polygons
% and associated detAreaHandles
function [detArea, bbox] = helperSaveDetectionArea(axesDet,detAreaHandles)
    pos = [];
    imageHandle = [];
    for k = 1:numel(detAreaHandles)
        pos = [pos; get(detAreaHandles(k),'Position')];
    end
    pos_min = min(pos);
    pos_max = max(pos);
    res = 50;
    img_min = floor(pos_min*res);
    img_max = ceil(pos_max*res);
    pos_min = img_min/res;
    pos_max = img_max/res;
    bbox = [pos_min pos_max-pos_min];
    I = zeros(uint8(bbox(4)*res),uint8(bbox(3)*res),'uint8');
    hold on
    imageHandle = image(axesDet,[bbox(1) bbox(1)+bbox(3)],[bbox(2) bbox(2)+bbox(4)],I,...
        "Visible","on");
    detArea = I;
    [nr, nc] = size(I);
    for k = 1:numel(detAreaHandles)
        r = floor(((nr-1)/bbox(4))*(detAreaHandles(k).Position(:,2) - pos_min(2)));
        c = floor(((nc-1)/bbox(3))*(detAreaHandles(k).Position(:,1) - pos_min(1)));
        detArea = detArea + uint8(poly2mask(c,r,nr,nc));
    end
end