use std::{
    io::{Read, Write},
    net::TcpListener,
    sync::mpsc,
    thread,
    time::Duration,
};
use bincode::{error::DecodeError, config::Configuration};
use rover_msgs::Message;

const TCP_PORT: u16 = 8000;
const SLEEP_DURATION: Duration = Duration::from_millis(10);
const BUFFER_SIZE: usize = 1024; 

#[derive(Debug)]
enum RoverLinkError {
    IoError(std::io::Error),
    BincodeError(bincode::error::EncodeError),
    DecodeError(bincode::error::DecodeError),
    BufferTooSmall,
}

impl From<std::io::Error> for RoverLinkError {
    fn from(err: std::io::Error) -> Self {
        RoverLinkError::IoError(err)
    }
}

impl From<bincode::error::EncodeError> for RoverLinkError {
    fn from(err: bincode::error::EncodeError) -> Self {
        RoverLinkError::BincodeError(err)
    }
}

impl From<bincode::error::DecodeError> for RoverLinkError {
    fn from(err: bincode::error::DecodeError) -> Self {
        RoverLinkError::DecodeError(err)
    }
}


struct RoverConnection {
    socket: std::net::TcpStream,
    read_buffer: [u8; BUFFER_SIZE],
    read_pos: usize,
    bytes_read: usize,
}

impl RoverConnection {
    fn new(socket: std::net::TcpStream) -> Result<Self, std::io::Error> {
        socket.set_nonblocking(true)?;
        Ok(RoverConnection {
            socket,
            read_buffer: [0; BUFFER_SIZE],
            read_pos: 0,
            bytes_read: 0,
        })
    }

    fn send_message(&mut self, msg: Message, config: Configuration) -> Result<(), RoverLinkError> {
        tracing::debug!("Sending message: {:?}", msg);
        bincode::serde::encode_into_std_write(msg, &mut self.socket, config)?;
        Ok(())
    }

    fn fill_buffer(&mut self) -> Result<bool, std::io::Error> {
        // Reset buffer if we've consumed all data
        if self.read_pos >= self.bytes_read {
            self.read_pos = 0;
            self.bytes_read = 0;
        }

        // Compact buffer if needed
        if self.read_pos > 0 && self.bytes_read > 0 {
            self.read_buffer.copy_within(self.read_pos..self.bytes_read, 0);
            self.bytes_read -= self.read_pos;
            self.read_pos = 0;
        }

        // Try to fill the remaining buffer space
        match self.socket.read(&mut self.read_buffer[self.bytes_read..]) {
            Ok(0) => Ok(false), // Connection closed
            Ok(n) => {
                self.bytes_read += n;
                Ok(true)
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(true),
            Err(e) => Err(e),
        }
    }

    fn receive_message(&mut self, config: Configuration) -> Result<Option<Message>, RoverLinkError> {
        // Try to fill buffer with new data
        if !self.fill_buffer()? {
            return Ok(None);
        }

        // Check if we have any data to process
        if self.read_pos >= self.bytes_read {
            return Ok(None);
        }

        // Try to decode a message from the buffer
        let available_slice = &self.read_buffer[self.read_pos..self.bytes_read];
        match bincode::serde::decode_from_slice(available_slice, config) {
            Ok((msg, consumed)) => {
                self.read_pos += consumed;
                tracing::debug!("Received message: {:?}", msg);
                Ok(Some(msg))
            }
            Err(DecodeError::Io { inner, .. }) if inner.kind() == std::io::ErrorKind::UnexpectedEof => {
                Ok(None) // Need more data
            }
            Err(e) => {
                tracing::debug!("Failed to decode message: {}", e);
               Ok(None)
            }
        }
    }
}

pub fn launch_rover_link(
    mut msg_rx: mpsc::Receiver<Message>,
) -> Result<(), Box<dyn std::error::Error>> {
    let listener = TcpListener::bind(("0.0.0.0", TCP_PORT))?;
    tracing::info!("Rover link launched on port {}", TCP_PORT);
    let config = bincode::config::standard();

    while let Ok((socket, addr)) = listener.accept() {
        tracing::info!("New connection: {}", addr);
        
        let mut connection = match RoverConnection::new(socket) {
            Ok(conn) => conn,
            Err(e) => {
                tracing::warn!("Failed to establish connection: {}", e);
                continue;
            }
        };

        loop {
            // Handle outgoing messages
            match msg_rx.try_recv() {
                Ok(msg) => {
                    if let Err(e) = connection.send_message(msg, config) {
                        tracing::error!("Failed to send message: {:?}", e);
                        break;
                    }
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => return Ok(()),
            }

            // Handle incoming messages
            match connection.receive_message(config) {
                Ok(Some(msg)) => {
                    // Handle received message here if needed

                }
                Ok(None) => {
                    thread::sleep(SLEEP_DURATION);
                    continue;
                }
                Err(e) => {
                    tracing::error!("Connection error: {:?}", e);
                    break;
                }
            }
        }
    }

    Ok(())
}