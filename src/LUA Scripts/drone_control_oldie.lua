function avance_callback(msg)
    -- This is the subscriber callback function
    avance = msg.data
end

function avance_eu_callback(msg)
    -- This is the subscriber callback function
    avance_eu = msg.data
end
function tankMass_callback(msg)
    tankMass = msg.data/1000
end
function axisForces_callback(msg)
    deltas = msg.data
end
function wayPoints_callback(msg)
    wayPoint_X = msg.data[3]
    wayPoint_Y = msg.data[4]
    wayPoint_Z = msg.data[5]
    
    route_num = msg.data[7]
    --print(msg.data)
end

function restart_callback(msg)
    restart = msg.data
    --print(restart)
end

function sysCall_init() 
    particlesAreVisible=false
    simulateParticles=false
    fakeShadow=false
    
    particleCountPerSecond=430
    particleSize=0.005
    particleDensity=8500
    particleScatteringAngle=30
    particleLifeTime=0.5
    maxParticleCount=50
    
    -- Create Python API Connection
    simRemoteApi.start(19998)
    rosInterfacePresent=simROS
    
    -- Prepare publisher and subscriber:
    if rosInterfacePresent then
        --Publishers
        publisher=simROS.advertise('/simulationTime','std_msgs/Float32')
        publisher2=simROS.advertise('/realTime','std_msgs/Float32')
        publisher_force=simROS.advertise('/force','std_msgs/Float32MultiArray')
        publisher_torque=simROS.advertise('/torque','std_msgs/Float32MultiArray')
        publisher_velocity=simROS.advertise('/velocity','std_msgs/Float32MultiArray')
        publisher_pose=simROS.advertise('/drone_pose','std_msgs/Float32MultiArray')
        publisher_TargetPose=simROS.advertise('/target_pose','std_msgs/Float32MultiArray')
        publisher_ori=simROS.advertise('/drone_orientation','std_msgs/Float32MultiArray')
        
        --Subscribers
        subs_axisForce = simROS.subscribe('/drone_axisForces','std_msgs/Float32MultiArray','axisForces_callback')
        subs_mass = simROS.subscribe('/currentMass', 'std_msgs/Float32', 'tankMass_callback')
        subs_avance=simROS.subscribe('/drone_nextPose','std_msgs/Float32MultiArray','avance_callback')
        subs_avance_eu=simROS.subscribe('/drone_nextEuler','std_msgs/Float32MultiArray','avance_eu_callback')
        subs_wayPoints=simROS.subscribe('/PE/Drone/drone_status','std_msgs/Float32MultiArray','wayPoints_callback')
        subs_restart=simROS.subscribe('/PE/Drone/restart','std_msgs/Bool','restart_callback')
    end

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadcopter_target')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal.
    d=sim.getObjectHandle('Quadcopter_base')
    wayPoint_handle=sim.getObjectHandle('wayPoint')

    propellerHandles={}
    jointHandles={}
    particleObjects={-1,-1,-1,-1}
    local ttype=sim.particle_roughspheres+sim.particle_cyclic+sim.particle_respondable1to4+sim.particle_respondable5to8+sim.particle_ignoresgravity
    if not particlesAreVisible then
        ttype=ttype+sim.particle_invisible
    end
    for i=1,4,1 do
        propellerHandles[i]=sim.getObjectHandle('Quadcopter_propeller_respondable'..i)
        jointHandles[i]=sim.getObjectHandle('Quadcopter_propeller_joint'..i)
        if simulateParticles then
            particleObjects[i]=sim.addParticleObject(ttype,particleSize,particleDensity,{2,1,0.2,3,0.4},particleLifeTime,maxParticleCount,{0.3,0.7,1})
        end
    end
    heli=sim.getObjectHandle(sim.handle_self)
    initialInertia,initialMatrix=sim.getShapeInertia(heli)
    
    masa = 26   -- Kg, Peso de solo el drone
    tankMass = 0 -- Masa variable del tanque, entra por el callback
    sim.setShapeMass(heli,masa)
    --Outer Loop Gains
    kp_r, kp_p = 4, 30
    
    --Controller Gains
    pParam=64
    iParam=0
    dParam=1
    
    pRoll, iRoll, dRoll = 18,0,0.05
    pPitch, iPitch, dPitch = 72,0,0.05
    
    cumul=0
    cumulRoll = 0
    cumulPitch = 0
    lastE=0
    lastERoll=0
    lastEPitch=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0
    
    prevEuler=0
    restart = false 
    
    thrust = 0
    alphaCorr = 0
    betaCorr = 0
    rotCorr = 0
    forces_array = {0,0,0,0}
    torque_array = {}
    deltas = {0,0,0}
    
    wayPoint_X = 0
    wayPoint_Y = 0
    wayPoint_Z = 0
    
    avance = sim.getObjectPosition(d,-1)
    avance_eu = sim.getObjectOrientation(d,-1)

    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end
end

function sysCall_cleanup() 
    --sim.removeDrawingObject(shadowCont)

    sim.setShapeMass(heli,masa)
    sim.setShapeInertia(heli,initialInertia,initialMatrix)
    
    sim.setArrayParam(sim.arrayparam_gravity,{0.0,0.0,-9.81})
    
    for i=1,#particleObjects,1 do
        sim.removeParticleObject(particleObjects[i])
    end
        -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher)
        simROS.shutdownPublisher(publisher2)
        simROS.shutdownPublisher(publisher_force)
        simROS.shutdownPublisher(publisher_torque)
        simROS.shutdownPublisher(publisher_pose)
        simROS.shutdownPublisher(publisher_TargetPose)
        simROS.shutdownPublisher(publisher_ori)
        simROS.shutdownPublisher(publisher_velocity)
    end
end 

function sysCall_actuation() 
    
    --If during the path the inertia moment gets too low, reset its dynamic and continue flying.
    actualInertia,actualMatrix=sim.getShapeInertia(heli)
    if actualInertia[1] < 1.5 then
        --print("TOO LOW! ----RESETING DYNAMICS")
        sim.setShapeMass(heli,masa)
        sim.setShapeInertia(heli,initialInertia,initialMatrix)
    end
    
    --print(sim.getShapeInertia(heli))
    sim.setShapeMass(heli,masa + 10)
    
    sim.setObjectPosition(targetObj,-1,avance)
    sim.setObjectPosition(wayPoint_handle,-1,{wayPoint_X,wayPoint_Y,wayPoint_Z})

    target_lin, target_ang = sim.getVelocity(targetObj)
    
    pos=sim.getObjectPosition(d,-1)
    ori=sim.getObjectOrientation(d,-1)
    
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    
    deltass = sim.getObjectPosition(heli,targetObj)
    
    --Position Controller Outer Loop
    desired_vel_x = deltass[1]*math.cos(ori[3]) + deltass[2]*math.sin(ori[3]) 
    desired_vel_y = deltass[2]*math.cos(ori[3]) - deltass[1]*math.sin(ori[3]) 
    desired_roll = kp_r*desired_vel_y
    desired_pitch = kp_p*desired_vel_x * -1
    
    if desired_roll > 0.8 then
        desired_roll = 0.8
    elseif desired_roll < -0.8 then
        desired_roll = -0.8
    end
    if desired_pitch > 15 then
        desired_pitch = 15
    elseif desired_pitch < -15 then
        desired_pitch = -15
    end
    deltaRoll = desired_roll - ori[1]
    deltaPitch = desired_pitch - ori[2]
    
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    --targetPos = avance
    pos=sim.getObjectPosition(d,-1)
    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    --thrust=5.45+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    thrust = (masa + 10) * (pv + iParam*cumul + dParam*(e-lastE))
    lastE=e
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]
    
    --Stabilization Control
    cumulRoll=cumulRoll+deltaRoll
    alphaCorr = ((pRoll*deltaRoll + iRoll*cumulRoll + dRoll*(deltaRoll-lastERoll))*-1)
    lastERoll = deltaRoll

    cumulPitch=cumulPitch+deltaPitch
    betaCorr = ((pPitch*deltaPitch + iPitch*cumulPitch + dPitch*(deltaPitch-lastEPitch))*-1)
    lastEPitch = deltaPitch
    
    forces_array[1] = thrust-alphaCorr+2*betaCorr+rotCorr
    forces_array[2] = thrust-alphaCorr-2*betaCorr-rotCorr
    forces_array[3] = thrust+alphaCorr-2*betaCorr+rotCorr
    forces_array[4] = thrust+alphaCorr+2*betaCorr-rotCorr
    
    for j=1,4,1 do
        if forces_array[j] < 0 then
            forces_array[j] = 0
        end
    end
    
    print(thrust, pos[3])
    print('others: ')
    print(alphaCorr)
    print(betaCorr)
    
    print('desired pitch: ', desired_pitch)
    print('deltapitch: ', deltaPitch)
    
    print('Array: ')
    print(forces_array)
    
    -- Decide of the motor velocities:
    handlePropeller(1,forces_array[1])
    handlePropeller(2,forces_array[2])
    handlePropeller(3,forces_array[3])
    handlePropeller(4,forces_array[4])
    
    torque_array[1] = torques(1, forces_array[1])
    torque_array[2] = torques(2, forces_array[2])
    torque_array[3] = torques(3, forces_array[3])
    torque_array[4] = torques(4, forces_array[4])
    
    --Se Publican los parametros necesarios
    if rosInterfacePresent then
        simROS.publish(publisher,{data=sim.getSimulationTime()})
        simROS.publish(publisher2,{data=sim.getSystemTime()})
        simROS.publish(publisher_pose,{data=sim.getObjectPosition(d,-1)})
        simROS.publish(publisher_TargetPose,{data=sim.getObjectPosition(targetObj,-1)})
        simROS.publish(publisher_ori,{data=sim.getObjectOrientation(d,-1)})
        simROS.publish(publisher_velocity,{data=target_lin})
        simROS.publish(publisher_force,{data=forces_array})
        simROS.publish(publisher_torque,{data=torque_array})
    end

end 


function handlePropeller(index,particleVelocity)
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0

    local t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    
    m=sim.getObjectMatrix(propeller,-1)
    particleCnt=0
    pos={0,0,0}
    dir={0,0,1}
    
    -- Apply a reactive force onto the body:
    --totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    totalExertedForce=particleVelocity
    force={0,0,totalExertedForce}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)
    local rotDir=1-math.mod(index,2)*2
    local total_torque = rotDir*0.002*particleVelocity
    torque={0,0,total_torque} --Me permite la rotacion
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)
    
end
function torques(index, particleVelocity)
    local rotDir=1-math.mod(index,2)*2
    local total_torque = rotDir*0.002*particleVelocity
    return total_torque
end