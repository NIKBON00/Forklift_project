function [angle] = angle_trilateration(antenna_positions,robots_tags,pos_est,distance_rs,idx)

tags = zeros(2,2);
options = optimset('Display','off');

for kkk = 1:2
    tags(kkk,:) = fminsearch(@(p) sum((sqrt(sum((antenna_positions - p).^2, 2)) - [distance_rs(idx(1)); distance_rs(idx(2)); distance_rs(idx(3))]).^2), pos_est' + robots_tags(kkk, :), options);
end

vec1 = tags(2,:) - tags(1,:);
angle = atan2(vec1(2) , vec1(1));

end