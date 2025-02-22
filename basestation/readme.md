# Redesigned Basestation

This project represents a redesigned basestation with the following key changes:

*   **Server Rewritten in Golang:** The backend server logic has been reimplemented using the Go programming language.
*   **Fake-Rover Rewritten in Python:** The simulated rover component has been rewritten in Python.
*   **Added Devcontainer:** A development container has been integrated to provide a consistent and reproducible development environment.

## How to Setup

1.  **Install VS Code Devcontainer Extension:** Ensure you have the "Dev Containers" extension installed in Visual Studio Code.  You can find it in the VS Code Marketplace.

2.  **Open this Folder as Devcontainer:** Open the root folder of this project in VS Code.  Then, use the "Dev Containers: Reopen in Container" command (press `Ctrl+Shift+P` or `Cmd+Shift+P` and type "Dev Containers").

3.  **Wait for Container Build:** The first time you open the project in the devcontainer, it may take a while to build the container.  This is normal, as it needs to download the base image and install all dependencies. Subsequent openings will be much faster.

## Building UI

1.  **Navigate to the `ui` Folder:** Open a terminal and navigate to the `ui` directory:

    ```bash
    cd ui
    ```

2.  **Build the UI:** Run the following command to build the user interface:

    ```bash
    npm run build
    ```

3.  **Auto-Rebuild (Optional):**  For automatic rebuilding of the UI whenever you make changes to the source code, you can use `watchexec`.  Install it if you don't have it:

    ```bash
    npm install -g watchexec
    ```

    Then, run:

    ```bash
    watchexec -e js,css,html,ts,tsx npm run build
    ```

    This will monitor the specified file extensions (`.js`, `.css`, `.html`, `.ts`, `.tsx`) and automatically rebuild the UI whenever a file with one of these extensions is modified.

## Running Server

1.  **Ensure UI is Built:** Make sure you have built the UI before running the server.  The server relies on the built UI files.

2.  **Navigate to the Root Directory:** Open a terminal and navigate to the root directory of the project (where the `main.go` file is located).

3.  **Run the Server:** Execute the following command to start the Go server:

    ```bash
    go run main.go
    ```

## Running Fake-Rover

1.  **Navigate to the `fake-rover` Directory:** Open a terminal and navigate to the `fake-rover` directory.

    ```bash
    cd fake-rover
    ```

2.  **Run the Fake-Rover Script:** Execute the following command to start the Python Fake-Rover script:

    ```bash
    python rover.py
    ```