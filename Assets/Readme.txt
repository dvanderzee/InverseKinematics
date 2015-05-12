Inverse Kinematics by David Vanderzee & Joseph Eduardo
	
	Made for Game Architecture Final Project



To run:
	To run our Inverse Kinematics simulation, open our project in unity. Load scene and play, IK.unity. Onload, the robot will begin reaching for a target desination via one of our algorithms. Hit up and down on the keyboard to switch between Inverse Kinematic solutions. The name of the solution is displayed in the top left.



Overview:
	For our project we implemented three solutions for inverse kinematics:
		Jacobian PsuedoInverse
		Damped Least Squares
		Cyclic Coordinate Descent

	The robot in our scene has one "arm" with two joints and an end effector. In the solutions the goal is for the robots "hand" to touch the target.
	Jacobian PsuedoInverse works by building a matrix of all the angles and vectors of the joints in the system, inverting it, and then multiplying the matrix by a goal vector. Each frame, each rotation value is updated slightly to bring it closer and closer to the goal.
	Damped Least Squares acts very similar to the Jacobian Inverse and actually needs to compute a Jacobian. It effects the rotation values of the joints in a very similar way. Damped Least Squares takes much longer to reach the goal.
	Cyclic Coordinate Descent is different from the previous solutions by rotating one joint at a time and traversing down the joint tree. It also does not need any use of matrices. CCD is much for jaggy then the other two but is still very precise.


Problems:

	There is an enormous lack of well documented tutorials on the Jacobian Inverse for 3D. Especially in any terms that were similar for this project. Most documentation we found wasn't well explained, leaving us wondering what most variables represented and how they pretained to us. Most examples were also in 2D which was still a problem because of a lack of good documentation. 
	Our project for Jacobian went through several iterations of changing the formula until we deduced how formulas online related to us. Relating the jacobians formulas to unity's 3D space was also confusing because of the change in axis's and directions.
	A very large chunk of our time went into trying to incorporate realistic and good working constraints on the system so it could behave like a normal hand. We origionally tried to clamp the rotation values based on min and max rotation values that a normal human can do. We now know that clamping roations that go out of bounds with the Jacobian don't satisfy all constraints either. In order to satisfy constraints for the Jacobian Inverse, you need to do several other functions for the joints to set up a new Jacobian Matrix to be used in the function instead of clamping values after the computation. Even with this method the result could break at any time. In the end, we decided to leave the constraints off so that the solutions could preform in a pure manner to show the exact algorithm.


Credits:

	Jacobian Inverse ----------	 David Vanderzee
	Damped Least Squares--------Joe Eduardo
	Cyclic Coordinate Descent---David Vanderzee & Joe Eduardo
	Unity Scene & Rigging-------David Vanderzee & Joe Eduardo

