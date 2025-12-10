import React, { useState, useEffect } from 'react';

interface Joint {
  id: string;
  name: string;
  min: number;
  max: number;
  value: number;
}

interface RobotControllerProps {
  robotName: string;
  joints: Joint[];
  onJointChange?: (joints: Joint[]) => void;
  className?: string;
}

const RobotController: React.FC<RobotControllerProps> = ({ 
  robotName, 
  joints: initialJoints, 
  onJointChange, 
  className = '' 
}) => {
  const [joints, setJoints] = useState<Joint[]>(initialJoints);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    // In a real implementation, this would set up connection to actual robot
    // For now, we'll just simulate connection
    console.log(`Simulating connection to ${robotName}`);
    
    return () => {
      console.log(`Disconnecting from ${robotName}`);
    };
  }, [robotName]);

  const handleJointChange = (id: string, value: number) => {
    const updatedJoints = joints.map(joint => 
      joint.id === id ? { ...joint, value } : joint
    );
    
    setJoints(updatedJoints);
    
    if (onJointChange) {
      onJointChange(updatedJoints);
    }
  };

  const connectToRobot = () => {
    // In a real implementation, this would connect to the actual robot
    setIsConnected(true);
    console.log(`Connected to ${robotName}`);
  };

  const disconnectFromRobot = () => {
    // In a real implementation, this would disconnect from the actual robot
    setIsConnected(false);
    console.log(`Disconnected from ${robotName}`);
  };

  return (
    <div className={`robot-controller ${className}`}>
      <div className="robot-header">
        <h3>{robotName} Controller</h3>
        <div className="connection-status">
          <span className={isConnected ? 'connected' : 'disconnected'}>
            {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
          </span>
          <button onClick={isConnected ? disconnectFromRobot : connectToRobot}>
            {isConnected ? 'Disconnect' : 'Connect'}
          </button>
        </div>
      </div>
      
      <div className="joints-panel">
        {joints.map((joint) => (
          <div key={joint.id} className="joint-control">
            <label htmlFor={`joint-${joint.id}`}>
              {joint.name}: {joint.value.toFixed(2)}°
            </label>
            <input
              id={`joint-${joint.id}`}
              type="range"
              min={joint.min}
              max={joint.max}
              value={joint.value}
              onChange={(e) => handleJointChange(joint.id, parseFloat(e.target.value))}
              className="joint-slider"
            />
            <span className="joint-range">{joint.min}° to {joint.max}°</span>
          </div>
        ))}
      </div>
      
      <div className="controller-actions">
        <button onClick={() => setJoints(initialJoints)}>Reset to Home</button>
        <button>Save Position</button>
        <button>Execute Trajectory</button>
      </div>
    </div>
  );
};

export default RobotController;