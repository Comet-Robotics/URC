import * as THREE from 'three';
import { createRoot } from 'react-dom/client';
import React, { useRef, useState, useMemo } from 'react';
import { Canvas, useFrame, ThreeElements, Vector3 } from '@react-three/fiber';
import { IMUData, Quaternion } from '@/types/binding';


interface BoxProps  {
  orientation: Quaternion;
  position: Vector3
}


function Box({ orientation, ...props }: BoxProps) {
  const meshRef = useRef<THREE.Mesh>(null!);
  const [hovered, setHover] = useState(false);
  const [active, setActive] = useState(false);

  // Use useMemo to avoid recreating the quaternion on every render
  const quaternion = useMemo(() => {
    return new THREE.Quaternion(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w
    );
  }, [orientation.x, orientation.y, orientation.z, orientation.w]); // Depend on individual quaternion components

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.rotation.setFromQuaternion(quaternion);
    }
  });

  return (
    <mesh
      {...props}
      ref={meshRef}
      scale={active ? 1.5 : 1}
      onClick={(event) => setActive(!active)}
      onPointerOver={(event) => setHover(true)}
      onPointerOut={(event) => setHover(false)}
    >
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color={hovered ? 'hotpink' : '#2f74c0'} />
    </mesh>
  );
}

export const IMUDisplay = ({ imu }: { imu?: IMUData }) => {
  return (
    <div
      className="w-80 h-80 rounded-[50%] "
      style={{
        border: '5px solid teal',
        overflow: 'hidden',
        background: 'rgba(0, 0, 0, 0.6)',
      }}
    >{
      <Canvas>
        <ambientLight intensity={Math.PI / 2} />
        <spotLight
          position={[10, 10, 10]}
          angle={0.15}
          penumbra={1}
          decay={0}
          intensity={Math.PI}
        />
        <pointLight position={[-10, -10, -10]} decay={0} intensity={Math.PI} />
        {
            imu?.orientation != undefined && 
        <Box position={[0, 0, 0]} orientation={imu.orientation} />
        }
      </Canvas>
    }
    </div>
  );
};