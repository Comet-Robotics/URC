import React from "react";
import VideoStream from "../components/VideoStream";
import Telemetry from "../components/Telemetry";
import Controller from "../components/Controller";
import {  useEffect, useState } from "react";
import useWebSocket from "react-use-websocket"
import { decodeMessage, GPSData, IMUData, Message, Twist, encodeMessage } from "@/types/binding";
import Connection from "@/components/Connection";

export default function HomePage() {
  const [socketUrl] = useState('/ws');
  const [gps, setGPS ] = useState<GPSData>();
  const [imu, setIMU] = useState<IMUData>();
  const [roverAddress,setRoverAddress] = useState<string>();
  const [message, setMessage] = useState<RTCIceCandidate | RTCSessionDescription>();
  const { sendMessage, lastMessage } = useWebSocket(socketUrl);

  useEffect(() => {
    const processMessage = async () => {
      if (lastMessage !== null) {
        // console.log(lastMessage);
        
        if (lastMessage.data instanceof Blob) {
          const arrayBuffer = await lastMessage.data.arrayBuffer();
          const uint8Array = new Uint8Array(arrayBuffer);
          const message: Message = decodeMessage(uint8Array);
          if (message.gps) {
            setGPS(message.gps);
          }
          if (message.imu) {
            setIMU(message.imu);
          }
        }else{
        
          
          const lastMessageData = JSON.parse(lastMessage.data)
          if (lastMessageData.roverAddress !== undefined){
            if (lastMessageData.roverAddress === false){
              setRoverAddress(undefined)
            }else{
              setRoverAddress(lastMessageData.roverAddress as string)
            }
          }else{
            setMessage(lastMessageData)
          }
        }
       
      }
    };
  
    processMessage();
  }, [lastMessage]);

  // const connectionStatus = {
  //   [ReadyState.CONNECTING]: 'Connecting',
  //   [ReadyState.OPEN]: 'Open',
  //   [ReadyState.CLOSING]: 'Closing',
  //   [ReadyState.CLOSED]: 'Closed',
  //   [ReadyState.UNINSTANTIATED]: 'Uninstantiated',
  // }[readyState];

  const handleMovement = (twist: Twist) => {
    // Create a Message object containing the twist
    const message: Message = {
      twist: twist
    };
    
    // Encode the full message
    const encoded = encodeMessage(message);
    
    // // Create a length-delimited message by prepending the length
    const lengthDelimited = new Uint8Array(encoded.length + 4);
    const view = new DataView(lengthDelimited.buffer);
    
    // Write the length as a 32-bit integer
    view.setUint32(0, encoded.length, true);
    
    // Copy the encoded message after the length
    lengthDelimited.set(encoded, 4);
    
    // Send the length-delimited message
    sendMessage(encoded);
  };

  return (
    <section className="grid xl:grid-cols-4 row-span-4 grid-cols-3 gap-4 p-4">
      <VideoStream   send_json={(value) => {sendMessage(JSON.stringify(value));}} message={message}  />
      <Connection roverAddress={roverAddress} />

      <Controller sendMovement={handleMovement} />
      <Telemetry gps={gps} imu={imu} />
    </section>
  );
}
