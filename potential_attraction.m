%Potential attarction force
function [Fatt] = potential_attraction(katt, current_pos, target_pos)
Fatt = katt*(target_pos-current_pos);