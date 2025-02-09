import React, { useState, useEffect } from "react";
import L from "leaflet";
import {
  MapContainer,
  TileLayer,
  Marker,
  useMap,
  Polyline,
  useMapEvents,
} from "react-leaflet";
import "leaflet/dist/leaflet.css";
import { Vector3 } from "../../../../rover-msgs/bindings/Vector3";

//==================================================================================================================================================================================

const createCustomIcon = (rotationAngle: number) =>
  L.divIcon({
    html: `<img src="https://pngimg.com/d/triangle_PNG76.png" style="transform: rotate(${rotationAngle}deg); width: 30px; height: 30px;" />`,
    className: "rover-icon",
    iconAnchor: [12.5, 65],
  });

const customMarkerIcon = L.icon({
  iconUrl:
    "https://cdn-icons-png.freepik.com/256/7976/7976248.png?semt=ais_hybrid",
  iconSize: [30, 30], // Adjust the size of the icon as needed
  iconAnchor: [15, 30], // Center the icon anchor for accurate placement
});

interface TrailPoint {
  position: [number, number];
  timestamp: number;
}

interface CustomMarker {
  position: [number, number];
}

const CenterMap: React.FC<{ position: [number, number] }> = ({ position }) => {
  const map = useMap();
  useEffect(() => {
    map.setView(position, map.getZoom()); // Center map position
  }, [map, position]);
  return null;
};

const ClickToAddMarker = ({ onMarkerAdd }:{
    onMarkerAdd: (position: [number, number]) => void;
  }) => {
  useMapEvents({
    click(e) {
      onMarkerAdd([e.latlng.lat, e.latlng.lng]); // Add marker at the clicked location
    },
  });
  return null;
};

//=====================================================================================================================================
// Position stuff
export const Map = ({roverPosition}:{roverPosition: [number, number]}) => {
  

  const [targetAngle, setTargetAngle] = useState<number>(0); // Target rotation angle
  const [currentAngle, setCurrentAngle] = useState<number>(0); // Smooth rotation angle

  //roration angle contant
  const [rotationAngle, setRotationAngle] = useState<number>(0);

  //trail
  const [trail, setTrail] = useState<TrailPoint[]>([]);

  //initial icon size
  const [iconSize, setIconSize] = useState<number>(30);

  const [markers, setMarkers] = useState<CustomMarker[]>([]);

  // Add the current position to the trail with offset
  useEffect(() => {
    setTrail((prevTrail) => [
      ...prevTrail,
      {
        position: [roverPosition[0] , roverPosition[1]],
        timestamp: Date.now(),
      },
    ]);
  }, [roverPosition]);

  // Remove points older than 5 seconds
  useEffect(() => {
    const interval = setInterval(() => {
      const currentTime = Date.now();
      setTrail((prevTrail) =>
        prevTrail.filter((point) => currentTime - point.timestamp <= 50000)
      );
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  // Add marker functionality
  const handleAddMarker = (position: [number, number]) => {
    setMarkers((prevMarkers) => {
      // If there are already 2 markers, remove the oldest one and add the new one
      if (prevMarkers.length >= 2) {
        return [...prevMarkers.slice(1), { position }];
      }
      // Otherwise, just add the new marker
      return [...prevMarkers, { position }];
    });
  };


  //==================================================================================================================================================================================

  return (
    <div
    className="w-80 h-80 rounded-[50%] "
      style={{
        border: "5px solid teal",
        overflow: "hidden",
        background: "rgba(0, 0, 0, 0.6)",
      
      }}
    >
      <MapContainer
        center={roverPosition}
        zoom={20}
        style={{ height: "400px", width: "100%", backgroundColor: "#171717" }}
        scrollWheelZoom={false}
        dragging={false}
        worldCopyJump={false}
        zoomControl={false}
        keyboard={false}
        doubleClickZoom={false}
      >
        <TileLayer
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          attribution="&copy; OpenStreetMap contributors"
        />
        {trail.map((point, index) => {
          const age = Date.now() - point.timestamp;
          let opacity = 1;

          if (age > 5000) {
            // Start fading out after 3 seconds
            opacity = 1 - (age - 5000) / 5000; // Fade out over 2 seconds
          }

          return (
            <Polyline
              positions={trail.map((point) => point.position)}
              color="lightblue"
              weight={4}
              opacity={0.8}
            />
          );
        })}
        {markers.map((marker, index) => (
          <Marker
            key={index}
            position={marker.position}
            icon={customMarkerIcon} // Use the custom icon here
          />
        ))}
        <ClickToAddMarker onMarkerAdd={handleAddMarker} />
        <Marker
          position={roverPosition}
          icon={createCustomIcon(rotationAngle)}
        />
        <CenterMap position={roverPosition} />
      </MapContainer>
    </div>
  );
};