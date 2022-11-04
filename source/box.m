function [P_bin,P_max,P_min,longline] = box(Point)

	for i = 1:3
		if i==1
			P_min(1) = min(Point(1,:));
			P_max(1) = max(Point(1,:));
		elseif i == 2
			P_min(2) = min(Point(2,:));
			P_max(2) = max(Point(2,:));
		else
			P_min(3) = min(Point(3,:));
			P_max(3) = max(Point(3,:));
		end
	end

	P_bin(1) = P_max(1)-P_min(1);
	P_bin(2) = P_max(2)-P_min(2);
	P_bin(3) = P_max(3)-P_min(3);
    
    %最小包围盒对角线长度
    longline = sqrt(sum(P_bin.^2));


end


