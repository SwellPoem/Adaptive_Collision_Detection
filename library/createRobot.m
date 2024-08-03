function robot = createRobot(DHTABLE, l)
    %% create the robot model
    dhparams = [DHTABLE(:,2) DHTABLE(:,1) DHTABLE(:,3:4)];
    robot = rigidBodyTree;
    body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('jnt1','revolute');

    setFixedTransform(jnt1,dhparams(1,:),'dh');
    body1.Joint = jnt1;

    addBody(robot,body1,'base');
    body2 = rigidBody('body2');
    jnt2 = rigidBodyJoint('jnt2','revolute');
    body3 = rigidBody('body3');
    jnt3 = rigidBodyJoint('jnt3','revolute');

    setFixedTransform(jnt2,dhparams(2,:),'dh');
    setFixedTransform(jnt3,dhparams(3,:),'dh');

    body2.Joint = jnt2;
    body3.Joint = jnt3;

    addBody(robot,body2,'body1');
    addBody(robot,body3,'body2');
end
