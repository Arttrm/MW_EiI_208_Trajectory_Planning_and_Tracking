# RC Car Modelling and Trajectory Tracking Control

Here is my attempt to MathWorks' Excellence in Innovation Project 208.
This work has been done as part of my MSc in Systems, Control and Signal processing at the University of Southampton

The best race driver is the fastest on track. However, "to finish first you must first finish" (Claude Rouelle). These objectives remain unchanged when dealing with remote control car racing. The objective of a race driver model is therefore to be as fast as possible on track while staying between the track limits.

This dissertation describes the development of a non-linear remote-controlled (RC) car model and its control system in order to test different trajectory optimisation strategies.

This project makes three contributions. Firstly, a high-fidelity non-linear vehicle model has been developed based on a four-wheel vehicle dynamics model and the Dugoff tyre model with adapted parameters to match the size and physics of a 1/10th remote-controlled car. This model provides a fair comparison basis for controller tuning and trajectory optimization.

Secondly, control systems for steering angle and speed have been developed to mimic the behaviour of a race driver. The steering controller is based on the pure pursuit algorithm. Classical control is then used to regulate the car speed to the speed target generated based on a friction circle model. The combination of the steering angle and speed controllers enabled to keep the car on the desired trajectory with a mean accuracy of a dozen of centimetres.

Thirdly, trajectory optimization has been studied. Before starting any trajectory optimization, the track centerline has been tracked to provide a measurement standard (Fig. 1 and Table I).

<table>
<tbody>
<td><figure>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Fig_Centerline_Tracking.png"  width=1000 />
    <figcaption>Fig. 1: Track centerline tracking result.</figcaption>
</figure></td>
<td><figure>
    <figcaption>Table I: Track centerline tracking result.</figcaption>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Tab_Centerline_Tracking.png"  width=500 />
</figure></td>
</tbody>
</table>


Finding the optimal trajectory can be seen as an optimization problem to solve the compromise between the shortest path (Fig. 2 and Table II) and the minimum curvature path (Fig. 3 and Table III). The resulting optimized trajectory (time optimal compromise) is given in Fig. 4 and Table IV.

<table>
<tbody>
<td><figure>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Fig_Shortest_Path_Tracking.png"  width=1000 />
    <figcaption>Fig. 2: Shortest path tracking result.</figcaption>
</figure></td>
<td><figure>
    <figcaption>Table II: Shortest path tracking result.</figcaption>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Tab_Shortest_Path_Tracking.png"  width=500 />
</figure></td>
</tbody>
<tbody>
<td><figure>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Fig_Minimum_Curvature_Path_Tracking.png"  width=1000 />
    <figcaption>Fig. 3: Minimum curvature path tracking result.</figcaption>
</figure></td>
<td><figure>
    <figcaption>Table III: Minimum curvature path tracking result.</figcaption>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Tab_Minimum_Curvature_Path_Tracking.png"  width=500 />
</figure></td>
</tbody>
<tbody>
<td><figure>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Fig_Offline_Optimal_Path_Tracking.png"  width=1000 />
    <figcaption>Fig. 4: Offline optimized path tracking result.</figcaption>
</figure></td>
<td><figure>
    <figcaption>Table IV: Offline optimized path tracking result.</figcaption>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Tab_Offline_Optimal_Path_Tracking.png"  width=500 />
</figure></td>
</tbody>
</table>

A further extension of this optimization problem is to propose an online implementation. The tracking of the online optimized path is given in the Fig. 5 and Table V.

<table>
<tbody>
<td><figure>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Gif_Online_Optimal_Path_Tracking.gif"  width=540 />
    <figcaption>Fig. 5: Online optimized path tracking result.</figcaption>
</figure></td>
<td><figure>
    <figcaption>Table V: Online optimized path tracking result.</figcaption>
    <img src="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/Figures/Tab_Online_Optimal_Path_Tracking.png"  width=200 />
</figure></td>
</tbody>
</table>

More details are given in my <a href="https://github.com/Arttrm/MW_EiI_208_Trajectory_Planning_and_Tracking/blob/main/MSc_Project_Dissertation.pdf">MSc project dissertation</a>.
