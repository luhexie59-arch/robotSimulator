# Robot Simulator - Pathfinding

### Overview
A Python-based robotic pathfinding simulator built with **Pygame**, featuring **A\*** and **D\*** algorithms, dynamic obstacle handling, and iterative path smoothing for realistic motion planning.

---

### Key Features
- **A\*** algorithm for efficient pathfinding  
- **D\*** algorithm for real-time replanning  
- **Dynamic obstacle detection** and response  
- **User interaction**: control obstacles manually or generate them randomly  
- **Iterative path smoothing** to mimic realistic robot motion  
- Modular and extendable structure for testing new search algorithms  

---

### Version Introduction
-	**Version 1**: Implements the A* algorithm to compute an optimal path from a start point to a target while avoiding stationary obstacles.
-	**Version 2**: Introduces a user-controlled moving obstacle, allowing real-time interaction via keyboard inputs to block or challenge the robot’s path.
-	**Version 3**: Replaces manual control with randomly generated moving obstacles, enhancing the simulation’s complexity and unpredictability.
-	**Latest versions (both 2 and 3)**: Adds an iterative path-smoothing method to eliminate redundant waypoints and generate more natural trajectories.

---

### How to Run
1. **Clone this repository**
2. **Install dependencies**
```bash
pip install pygame numpy
```
3. **Run the simulator**

---

### Known Issues
- Sometimes the robot may crash into mobile obstacles at turning points

### Future Improvements
- Use advanced algorithms, such as probabilistic pathfinding (e.g., RRT*)
- Update dynamic obstacle detecting algorithm
- Add curvatures at turnings
- Enable dynamic speed control
- Refactor some parts of the code to improve computation efficiency
- Bug fixes
---
### Example Screenshots
#### v2 (using map 5)
![v2example](https://raw.githubusercontent.com/luhexie59-arch/robotSimulator/refs/heads/main/assets/g2_4.png)
#### v3 (using map 6)
![v3example](https://raw.githubusercontent.com/luhexie59-arch/robotSimulator/refs/heads/main/assets/g1_3.png)
