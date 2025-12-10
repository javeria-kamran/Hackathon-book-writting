# Week 2: Setting Up Your Robotics Development Environment

## 2.1: Installing Ubuntu and Core Dependencies

A robust and consistent operating system is the foundation of any robotics development environment. Throughout this course, we will standardize on **Ubuntu 22.04 LTS (Jammy Jellyfish)** as our primary development OS. This specific version provides long-term support and is the recommended platform for ROS 2 Humble Hawksbill, NVIDIA Jetson, and Isaac SDK compatibility.

### Installation Steps:

1.  **Download Ubuntu 22.04 LTS:** Obtain the ISO image from the official Ubuntu website.
2.  **Create Bootable USB:** Use tools like Rufus (Windows) or Etcher (cross-platform) to create a bootable USB drive.
3.  **Install Ubuntu:** Boot from the USB and follow the on-screen instructions. It is recommended to perform a clean installation if possible. If dual-booting with Windows, ensure you allocate sufficient disk space (at least 200GB) for your Ubuntu partition.
4.  **Initial System Update:** After installation, open a terminal (Ctrl+Alt+T) and run the following commands to ensure your system is up-to-date:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    sudo apt autoremove -y
    ```
5.  **Install Essential Tools:** A few core development tools are indispensable:
    ```bash
    sudo apt install -y build-essential curl git nano vim htop cmake pkg-config
    ```
    *   `build-essential`: Provides compilers and development libraries.
    *   `curl`, `git`: For downloading and managing code.
    *   `nano`, `vim`: Command-line text editors.
    *   `htop`: System resource monitor.
    *   `cmake`, `pkg-config`: Build system tools.

Ensuring these fundamental steps are correctly executed will prevent many common setup issues later in the course.

## 2.2: Setting up NVIDIA Drivers and CUDA

For any serious AI or robotics development on NVIDIA hardware (especially for Isaac Sim and Jetson), proper installation of NVIDIA graphics drivers and the CUDA toolkit is paramount. CUDA is NVIDIA's parallel computing platform and API model that enables dramatic increases in computing performance by harnessing the power of the GPU.

### Installation Steps for NVIDIA Drivers (Desktop GPU):

1.  **Purge Existing Drivers (if any):** It's often best to start fresh.
    ```bash
    sudo apt purge -y "*nvidia*"
    ```
2.  **Add Graphics Drivers PPA:**
    ```bash
    sudo add-apt-repository ppa:graphics-drivers/ppa -y
    sudo apt update
    ```
3.  **Install Recommended Driver:**
    ```bash
    ubuntu-drivers install
    sudo reboot # Reboot for changes to take effect
    ```
    Alternatively, you can manually select a driver version. After reboot, verify installation with `nvidia-smi`.

### Installing CUDA Toolkit (for Desktop GPU):

While the NVIDIA drivers are crucial, the CUDA Toolkit provides the necessary development environment for GPU-accelerated computing.
1.  **Download CUDA Toolkit:** Visit the NVIDIA CUDA Toolkit Archive and select the appropriate version for Ubuntu 22.04. Follow the `deb (local)` installation instructions.
    *   *Example (adjust version as needed):*
        ```bash
        wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
        sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
        wget https://developer.download.nvidia.com/compute/cuda/12.2.2/local_installers/cuda-repo-ubuntu2204-12-2-local_12.2.2-1_amd64.deb
        sudo dpkg -i cuda-repo-ubuntu2204-12-2-local_12.2.2-1_amd64.deb
        sudo cp /var/cuda-repo-ubuntu2204-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
        sudo apt update
        sudo apt install -y cuda-toolkit-12-2 # Install the toolkit
        ```
2.  **Set Environment Variables:** Add the following lines to your `~/.bashrc` (or `~/.zshrc`) file:
    ```bash
    export PATH=/usr/local/cuda-12.2/bin${PATH:+:${PATH}}
    export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
    ```
    Then, apply changes: `source ~/.bashrc`.

Properly configured NVIDIA drivers and CUDA are non-negotiable for running Isaac Sim and other GPU-accelerated software that you'll use throughout this course.

## 2.3: Docker for Reproducible Environments

Docker is an essential tool for creating isolated, reproducible, and portable development environments. It encapsulates your application and its dependencies into a container, ensuring that your code runs consistently across different machines. For robotics, Docker helps manage complex software stacks like ROS 2 and simplifies dependency management.

### Why Docker for Robotics?

*   **Reproducibility:** Avoid "it works on my machine" issues by ensuring everyone uses the exact same environment.
*   **Isolation:** Prevent conflicts between different projects or software versions on your host system.
*   **Portability:** Easily move your development environment between different workstations or even cloud instances.
*   **Simplified Setup:** New team members or students can quickly get started without extensive manual installations.

### Installing Docker Engine:

Follow the official Docker documentation for the most up-to-date installation instructions for Ubuntu. Here's a summarized guide:

1.  **Uninstall Old Versions:**
    ```bash
    sudo apt-get remove docker docker-engine docker.io containerd runc
    ```
2.  **Install Using the Repository:**
    ```bash
    sudo apt-get update
    sudo apt-get install ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg
    echo "deb [arch=\"$(dpkg --print-architecture)\" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \"$(. /etc/os-release && echo \"$VERSION_CODENAME\")\" stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```
3.  **Manage Docker as a Non-Root User:**
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker # You might need to log out/in or reboot for this to take full effect
    ```
4.  **Verify Installation:**
    ```bash
    docker run hello-world
    ```

### NVIDIA Container Toolkit (NVIDIA Docker):

To allow Docker containers to access your NVIDIA GPU, you need to install the NVIDIA Container Toolkit. This is crucial for running GPU-accelerated applications inside Docker, such as Isaac Sim or ROS 2 packages that use CUDA.

1.  **Configure NVIDIA Container Toolkit:**
    ```bash
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/ubuntu22.04/libnvidia-container.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    ```
2.  **Restart Docker Daemon:**
    ```bash
    sudo systemctl restart docker
    ```
3.  **Test NVIDIA Docker:**
    ```bash
    docker run --rm --gpus all nvidia/cuda:12.2.2-base-ubuntu22.04 nvidia-smi
    ```
    You should see output similar to `nvidia-smi` from your host, indicating the GPU is accessible within the container.

Docker, combined with NVIDIA Container Toolkit, will be your go-to method for managing complex robotics development environments, ensuring consistency and portability across different systems.