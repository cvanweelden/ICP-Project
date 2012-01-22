function pc = apply_transform(pc, Q);
    pc = [pc; ones(1, size(pc,2))];
    pc = Q*pc;
    pc = pc(1:3,:);
end