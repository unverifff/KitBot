
    clf
    clc
    
    % r = LinearUR3;
    % hold on
    % % r.model.plot3d(q)   
    % % r.model.teach(q)
    % q = zeros(1,r.model.n);
    % r.model.teach(q)


    base_rotation = trotz(pi)  % No rotation for simplicit
    r1 = GripperRight();

    r2 = GripperLeft();

    view(3);
    qa = [0,0,0];
    qb = [0,deg2rad(60),0];
    qc = [0,deg2rad(-60),0];

    jMatrix = jtraj(qa, qb, 40); %%quintic polynomial, faster but lower resol
    iMatrix = jtraj(qa, qc, 40);

    for x = 1:length(jMatrix) 

        r1.model.animate(jMatrix(x, :));
        hold on
        r2.model.animate(iMatrix(x, :));
        drawnow();

    end

    xlim([-0.3 0.3])
    ylim([-0.3 0.3])
    zlim([0 0.3])

    tm12 = LinearTM12;

    qz = [0,0,0,0,0,0,0];
    axis equal;
    T = transl(0.9,0.1,0.9)

    q = zeros(1,7); 

    qn = tm12.model.ikcon(T)

    initQ = tm12.model.getpos(); %previous position
    steps = 40; %frames of motion to end effector

    s = lspb(0, 1, steps); 
    qSteps = nan(steps, 7);

    for w = 1:steps %%trapizodal velocity profile higher res but high computations
        qSteps(w, :) = (1 - s(w)) * initQ + s(w) * qn %%scalar interpolation function
    end

    jMatrix = jtraj(initQ, qn, steps);

    EFF = tm12.model.fkineUTS(tm12.model.getpos());

    for x = 1:length(jMatrix) 

        tm12.model.animate(jMatrix(x, :));
        EFF = tm12.model.fkineUTS(tm12.model.getpos());

        r1.model.base = EFF;
        r2.model.base = EFF;

        r1.model.animate(qb);
        hold on
        r2.model.animate(qc);

        drawnow();

    end
    axis equal;