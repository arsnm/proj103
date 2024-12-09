# Networking Documentation

## WebSocket Server

- Handles multiple client connections
- Routes messages between robot and interfaces
- Manages client state
- Provides HTTP server for web interface


## Communication with Tracking Server

- Robot send its latest estimated position to the server
- Update the `RaceController` status with the informations received from the server
- Robot send *new* detected markers to the server

## Security Considerations

- Single robot connection
- Client type verification
- Connection state tracking
- Message validation


## Performance

- Video compression
- Message queuing
- Automatic reconnection
- Error recovery

