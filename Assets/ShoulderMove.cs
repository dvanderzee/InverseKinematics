﻿using UnityEngine;
using System.Collections;

public class ShoulderMove : MonoBehaviour {

	// These floats will represent the Shoulder's
	// rotation in 3D space. They are public so 
	// they can be changed in the editor. The 
	// Shoulder has 3 degrees of freedom, so x, y,
	// and z axis are neccessary.
	public float xRot;
	public float yRot;
	public float zRot;

	// These are the min and max rotations the shoulder can achieve.
	// These values will be used for clamping rotation. They
	// are public so they can be changed in the editor.
	public float xMin, xMax;
	public float yMin, yMax;
	public float zMin, zMax;

	// Target for the end affector
	public Transform elbow;
	public Transform hand;
	public Transform target;


	// Matrices to hold our Jacobian Values and it's Adjacency Matrix
	float[,] jacobian = new float[3, 2];
	float[,] psuedo = new float[2, 3];

	// Speed at which objects are manually rotated
	public float rotSpeed;

	// Determinant used in inverting the matrix
	private float determinant;

	// Minimum Distance required to stop moving arm
	public float minDistance;

	// Boolean to keep calculating new Joints
	public bool done = false;

	public GameObject elbowObject;
	public float elbowX, elbowY, elbowZ;

	Vector3 shoulderAxis;		// Axis of Revolution for Shoulder
	Vector3 elbowAxis;			// Axis of Revolution for Elbow
	ElbowScript elbowscript;

	public float step;

	// Use this for initialization
	void Start () {

		elbowObject = GameObject.Find ("Elbow");
		elbowscript = elbowObject.GetComponent<ElbowScript> ();

	}

	// (Axis of Revolution) X (Vector from joint to end effector)
	void jacobianCalculation() {

		elbowX = elbowscript.xRot;
		elbowY = elbowscript.yRot;
		elbowZ = elbowscript.zRot;

		Vector3 shoulderRot = new Vector3 (xRot, yRot, zRot);		// Current Rotation Vector for Shoulder
		Vector3 elbowRot = new Vector3 (elbowX, elbowY, elbowZ);	// Current Rotation Vector for Elbow

		float shoulderRotMag = 	// Magnitude of the Shoulder Rotation
			Mathf.Sqrt (Mathf.Pow (xRot, 2) + Mathf.Pow (yRot, 2) + Mathf.Pow (zRot, 2));
		float elbowRotMag = 	// Magnitude of the Elbow Rotation
			Mathf.Sqrt (Mathf.Pow (elbowX, 2) + Mathf.Pow (elbowY, 2) + Mathf.Pow (elbowZ, 2));


		shoulderAxis = (shoulderRot / shoulderRotMag);
		elbowAxis = (elbowRot / elbowRotMag);

		Vector3 shoulderDist = (hand.transform.position - transform.position);		// Vector from Shoulder to Hand
		Vector3 elbowDist = (hand.transform.position - elbow.transform.position);	// Vector from Elbow to Hand

		Vector3 shoulderCol = Vector3.Cross(shoulderAxis, shoulderDist);			// Final Result for Shoulder
		Vector3 elbowCol = Vector3.Cross(elbowAxis, elbowDist);						// Final Result for Elbow

		// Entries into the Jacobian
		jacobian [0, 0] = shoulderCol.x;
		jacobian [1, 0] = shoulderCol.y;
		jacobian [2, 0] = shoulderCol.z;
		jacobian [0, 1] = elbowAxis.x;
		jacobian [1, 1] = elbowAxis.y;
		jacobian [2, 1] = elbowAxis.z;


		Debug.Log("Jacobian is:");
		Debug.Log("Col 1: " + jacobian[0,0] + " " + jacobian[1,0] + " " + jacobian[2,0]);
		Debug.Log("Col 2: " + jacobian[0,1] + " " + jacobian[1,1] + " " + jacobian[2,1]);

	}
	// PsuedoInverse = Inverse(Transpose * Jacobian) * Transpose
	void pseudoInverse() {
		float[,] transpose = new float[2, 3];

		transpose [0, 0] = jacobian [0, 0];
		transpose [0, 1] = jacobian [1, 0];
		transpose [0, 2] = jacobian [2, 0];
		transpose [1, 0] = jacobian [0, 1];
		transpose [1, 1] = jacobian [1, 1];
		transpose [1, 2] = jacobian [2, 1];
	
		float[,] temp = new float[2, 2];
		float[,] tempinv = new float[2, 2];

		temp [0, 0] = (transpose [0, 0] * jacobian [0, 0]) + (transpose [0, 1] * jacobian [1, 0]) + (transpose [0, 2] * jacobian [2, 0]);
		temp [0, 1] = (transpose [0, 0] * jacobian [0, 1]) + (transpose [0, 1] * jacobian [1, 1]) + (transpose [0, 2] * jacobian [2, 1]);
		temp [1, 0] = (transpose [1, 0] * jacobian [0, 0]) + (transpose [1, 1] * jacobian [1, 0]) + (transpose [1, 2] * jacobian [2, 0]);
		temp [1, 1] = (transpose [1, 0] * jacobian [0, 1]) + (transpose [1, 1] * jacobian [1, 1]) + (transpose [1, 2] * jacobian [2, 1]);

		float determinant = (temp [0, 0] * temp [1, 1]) - (temp [0, 1] * temp [1, 0]);
		tempinv [0, 0] = temp [1, 1] / determinant;
		tempinv [0, 1] = -temp [0, 1] / determinant;
		tempinv [1, 0] = -temp [1, 0] / determinant;
		tempinv [1, 1] = temp [0, 0] / determinant;

		psuedo [0, 0] = (tempinv [0, 0] * transpose [0, 0]) + (tempinv [0, 1] * transpose [1, 0]);
		psuedo [0, 1] = (tempinv [0, 0] * transpose [0, 1]) + (tempinv [0, 1] * transpose [1, 1]);
		psuedo [0, 2] = (tempinv [0, 0] * transpose [0, 2]) + (tempinv [0, 1] * transpose [1, 2]);
		psuedo [1, 0] = (tempinv [1, 0] * transpose [0, 0]) + (tempinv [1, 1] * transpose [1, 0]);
		psuedo [1, 1] = (tempinv [1, 0] * transpose [0, 1]) + (tempinv [1, 1] * transpose [1, 1]);
		psuedo [1, 2] = (tempinv [1, 0] * transpose [0, 2]) + (tempinv [1, 1] * transpose [1, 2]);



		Debug.Log("Jacobian Inverse is:");
		Debug.Log("Row 1: " + psuedo[0,0] + " " + psuedo[0,1] + " " + psuedo[0,2]);
		Debug.Log("Row 2: " + psuedo[1,0] + " " + psuedo[1,1] + " " + psuedo[1,2]);

	}

	void computeNewJoints() {
		Vector3 deltaTranslation = target.transform.position - hand.transform.position;
		if (Vector3.Distance (target.transform.position, hand.transform.position) < minDistance) {
			Debug.Log("target distance: " + Vector3.Distance (target.transform.position, hand.transform.position));
			done = true;
		}

		// Change in DOF
		float[,] deltaTheta = new float[2, 1];
		deltaTheta [0, 0] = (psuedo [0, 0] * deltaTranslation.x) + (psuedo [0, 1] * deltaTranslation.y) + (psuedo [0, 2] * deltaTranslation.z);
		deltaTheta [1, 0] = (psuedo [1, 0] * deltaTranslation.x) + (psuedo [1, 1] * deltaTranslation.y) + (psuedo [1, 2] * deltaTranslation.z);
	
		xRot += (deltaTheta [0, 0] * step);
		yRot += (deltaTheta [0, 0] * step);
		zRot += (deltaTheta [0, 0] * step);

		elbowscript.xRot += (deltaTheta [1, 0] * step);
		elbowscript.yRot += (deltaTheta [1, 0] * step);
		elbowscript.zRot += (deltaTheta [1, 0] * step);
	}

	// Update is called once per frame
	void Update () {

		if(done == false) {
			jacobianCalculation ();
			pseudoInverse();
			computeNewJoints();


			//xRot = Mathf.Clamp(xRot, xMin, xMax);
			//yRot = Mathf.Clamp(yRot, yMin, yMax);
			//zRot = Mathf.Clamp(zRot, zMin, zMax);

			//elbowscript.xRot = Mathf.Clamp(elbowscript.xRot, elbowscript.xMin, elbowscript.xMax);
			//elbowscript.yRot = Mathf.Clamp(elbowscript.yRot, elbowscript.yMin, elbowscript.yMax);
			//elbowscript.zRot = Mathf.Clamp(elbowscript.zRot, elbowscript.zMin, elbowscript.zMax);

			Debug.Log("Shoulder Rotations: " + xRot + " " + yRot + " " + zRot);
			Debug.Log("Elbow Rotations: " + elbowscript.xRot + " " + elbowscript.yRot + " " + elbowscript.zRot);

			transform.rotation = Quaternion.Euler(xRot,yRot,zRot);
			elbow.transform.rotation = Quaternion.Euler(elbowscript.xRot, elbowscript.yRot, elbowscript.zRot);
		}

	}
}
