using UnityEngine;
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
	float[,] jacobian = new float[3, 6];
	float[,] psuedo = new float[6, 3];

	// Speed at which objects are manually rotated
	public float rotSpeed;

	// Determinant used in inverting the matrix
	private float determinant;
	private float detPass;

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
		Vector3 xAxis = new Vector3 (1, 0, 0);
		Vector3 yAxis = new Vector3 (0, 1, 0);
		Vector3 zAxis = new Vector3 (0, 0, 1);

		Vector3 shoulderDist = (hand.transform.position - transform.position);		// Vector from Shoulder to Hand
		Vector3 elbowDist = (hand.transform.position - elbow.transform.position);	// Vector from Elbow to Hand

		Vector3 shoulderCol1 = Vector3.Cross (xAxis, shoulderDist);
		Vector3 shoulderCol2 = Vector3.Cross (yAxis, shoulderDist);
		Vector3 shoulderCol3 = Vector3.Cross (zAxis, shoulderDist);

		Vector3 elbowCol1 = Vector3.Cross (xAxis, elbowDist);
		Vector3 elbowCol2 = Vector3.Cross (yAxis, elbowDist);
		Vector3 elbowCol3 = Vector3.Cross (zAxis, elbowDist);

		jacobian[0,0] = shoulderCol1.x;
		jacobian[0,1] = shoulderCol2.x;
		jacobian[0,2] = shoulderCol3.x;
		jacobian[0,3] = elbowCol1.x;
		jacobian[0,4] = elbowCol2.x;
		jacobian[0,5] = elbowCol3.x;
		jacobian[1,0] = shoulderCol1.y;
		jacobian[1,1] = shoulderCol2.y;
		jacobian[1,2] = shoulderCol3.y;
		jacobian[1,3] = elbowCol1.y;
		jacobian[1,4] = elbowCol2.y;
		jacobian[1,5] = elbowCol3.y;
		jacobian[2,0] = shoulderCol1.z;
		jacobian[2,1] = shoulderCol2.z;
		jacobian[2,2] = shoulderCol3.z;
		jacobian[2,3] = elbowCol1.z;
		jacobian[2,4] = elbowCol2.z;
		jacobian[2,5] = elbowCol3.z;

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 6; j++) {
				Debug.Log ("Jacobian: " + jacobian[i,j]);
			}
		}
		
	}
	// PsuedoInverse = Inverse(Transpose * Jacobian) * Transpose
	void pseudoInverse() {
		float[,] transpose = new float[6, 3];

		//computes the transpose
		transpose [0, 0] = jacobian [0, 0];
		transpose [0, 1] = jacobian [1, 0];
		transpose [0, 2] = jacobian [2, 0];
		transpose [1, 0] = jacobian [0, 1];
		transpose [1, 1] = jacobian [1, 1];
		transpose [1, 2] = jacobian [2, 1];
		transpose [2, 0] = jacobian [0, 2];
		transpose [2, 1] = jacobian [1, 2];
		transpose [2, 2] = jacobian [2, 2];
		transpose [3, 0] = jacobian [0, 3];
		transpose [3, 1] = jacobian [1, 3];
		transpose [3, 2] = jacobian [2, 3];
		transpose [4, 0] = jacobian [0, 4];
		transpose [4, 1] = jacobian [1, 4];
		transpose [4, 2] = jacobian [2, 4];
		transpose [5, 0] = jacobian [0, 5];
		transpose [5, 1] = jacobian [1, 5];
		transpose [5, 2] = jacobian [2, 5];
	
		float[,] temp = new float[6, 6];
		float[,] tempinv = new float[6, 6];

		for (int row = 0; row < 6; row++) {
			for (int col = 0; col < 6; col++) {
				for (int inner = 0; inner < 3; inner++) {
					temp [row, col] += transpose [row, inner] * jacobian [inner, col];
				}
			}
		}

		determinant = deter (temp, 6);

		for (int i = 0; i < 6; i++) {
			for (int j = 0; j < 6; j++) {
				tempinv [i, j] = temp [j, i];
			}
		}

		for (int i = 0; i < 6; i++) {
			for(int j = 0; j < 6; j++) {
				tempinv[i,j] = tempinv[i,j] / determinant;
			}
		}


		// Calculating the PsuedoInverse
		for (int row = 0; row < 6; row++) {
			for (int col = 0; col < 3; col++) {
				for (int inner = 0; inner < 6; inner++) {
					psuedo[row,col] += tempinv[row,inner] * transpose[inner, col];
				}
			}
		}


		Debug.Log("Jacobian Inverse is:");
		Debug.Log("Row 1: " + psuedo[0,0] + " " + psuedo[0,1] + " " + psuedo[0,2]);
		Debug.Log("Row 2: " + psuedo[1,0] + " " + psuedo[1,1] + " " + psuedo[1,2]);

	}

	//compute change in DOF for Joints.
	// Delta Theta = 6x3Jacobian Inverse * 3x1deltaE
	void computeNewJoints() {
		
		Vector3 deltaTranslation = target.transform.position - hand.transform.position;
		
		//Have we reached our destination?
		if (Vector3.Distance (target.transform.position, hand.transform.position) < minDistance) {
			Debug.Log("target distance: " + Vector3.Distance (target.transform.position, hand.transform.position));
			done = true;
		}
		
		//Compute change in DOF
		float[,] deltaTheta = new float[6,1];
		deltaTheta[0,0] = (psuedo[0,0] * deltaTranslation.x) + (psuedo[0,1] * deltaTranslation.y) + (psuedo[0,2] * deltaTranslation.z);
		deltaTheta[1,0] = (psuedo[1,0] * deltaTranslation.x) + (psuedo[1,1] * deltaTranslation.y) + (psuedo[1,2] * deltaTranslation.z);
		deltaTheta[2,0] = (psuedo[2,0] * deltaTranslation.x) + (psuedo[2,1] * deltaTranslation.y) + (psuedo[2,2] * deltaTranslation.z);
		deltaTheta[3,0] = (psuedo[3,0] * deltaTranslation.x) + (psuedo[3,1] * deltaTranslation.y) + (psuedo[3,2] * deltaTranslation.z);
		deltaTheta[4,0] = (psuedo[4,0] * deltaTranslation.x) + (psuedo[4,1] * deltaTranslation.y) + (psuedo[4,2] * deltaTranslation.z);
		deltaTheta[5,0] = (psuedo[5,0] * deltaTranslation.x) + (psuedo[5,1] * deltaTranslation.y) + (psuedo[5,2] * deltaTranslation.z);
		
		
		//update Joints
		xRot += (deltaTheta[0,0] * step);
		yRot += (deltaTheta[1,0] * step);
		zRot += (deltaTheta[2,0] * step);
		
		elbowscript.xRot += (deltaTheta[3,0] * step);
		elbowscript.yRot += (deltaTheta[4,0] * step);
		elbowscript.zRot += (deltaTheta[5,0] * step);
		
	}

	float deter(float[,] a, int n) {
		int i, j, k;
		float det = 0;
		for (i = 0; i < n - 1; i++)
		{   
			for (j = i + 1; j < n; j++)
			{
				det = a[j, i] / a[i, i];
				for (k = i; k < n; k++)
					a[j, k] = a[j, k] - det * a[i, k]; // Here's exception
			}
		}
		det = 1;
		for (i = 0; i < n; i++)
			det = det * a[i, i];
		return det;

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
