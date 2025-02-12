import React from "react";
import VideoStream from "../components/VideoStream";
import Telemetry from "../components/Telemetry";
import Controller from "../components/Controller";
import Control from "@/components/Control";
import {  useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket"
import { decodeMessage, encodeTwist, GPSData, IMUData, Message, Quaternion, Vector3, Twist, encodeMessage } from "@/types/binding";

export default function HomePage() {
  const [socketUrl, setSocketUrl] = useState('/message_stream');
  const [messageHistory, setMessageHistory] = useState<Message[]>([]);
  const [gps, setGPS ] = useState<GPSData>();
  const [imu, setIMU] = useState<IMUData>();

  const { sendMessage, lastMessage, readyState } = useWebSocket(socketUrl);

  useEffect(() => {
    const processMessage = async () => {
      if (lastMessage !== null) {
        console.log(lastMessage);
        
        // If lastMessage.data is a Blob
        if (lastMessage.data instanceof Blob) {
          const arrayBuffer = await lastMessage.data.arrayBuffer();
          const uint8Array = new Uint8Array(arrayBuffer);
          let message: Message = decodeMessage(uint8Array);
          if (message.gps) {
            setGPS(message.gps);
          }
          if (message.imu) {
            setIMU(message.imu);
          }
        } 
       
      }
    };
  
    processMessage();
  }, [lastMessage]);

  const connectionStatus = {
    [ReadyState.CONNECTING]: 'Connecting',
    [ReadyState.OPEN]: 'Open',
    [ReadyState.CLOSING]: 'Closing',
    [ReadyState.CLOSED]: 'Closed',
    [ReadyState.UNINSTANTIATED]: 'Uninstantiated',
  }[readyState];

  const handleMovement = (twist: Twist) => {
    // Create a Message object containing the twist
    const message: Message = {
      twist: twist
    };
    
    // Encode the full message
    const encoded = encodeMessage(message);
    
    // // Create a length-delimited message by prepending the length
    // const lengthDelimited = new Uint8Array(encoded.length + 4);
    // const view = new DataView(lengthDelimited.buffer);
    
    // // Write the length as a 32-bit integer
    // view.setUint32(0, encoded.length, true);
    
    // // Copy the encoded message after the length
    // lengthDelimited.set(encoded, 4);
    
    // Send the length-delimited message
    sendMessage(encoded);
  };

  return (
    <section className="grid grid-cols-3 gap-4 p-4">
      <VideoStream/>
      <Controller sendMovement={handleMovement} />
      <Telemetry gps={gps} imu={imu}/>
    </section>
  );
}