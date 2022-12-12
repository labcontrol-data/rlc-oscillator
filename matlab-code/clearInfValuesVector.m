% clean 'Inf' values from vector

function [vecClean] = clearInfValuesVector(vec,TOL)

if sum(vec)==Inf
    vec_idx = find(vec==Inf);
    for i=1:max(size(vec_idx))
        idx = vec_idx(i);
        if (vec(idx-1)<TOL)&&(idx>1)
            vec(idx)=vec(idx-1);
        end
    end
end
vecClean = vec;