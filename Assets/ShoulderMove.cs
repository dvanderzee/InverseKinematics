using UnityEngine;
using System.Collections;
using CSML;

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
	//float[,] jacobian = new float[3, 6];
	//float[,] psuedo = new float[6, 3];

	Matrix jacobian = new Matrix (3, 9);
	Matrix pseudoInv = new Matrix (9, 3);

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
		Vector3 endDist = (hand.transform.position);

		Vector3 shoulderCol1 = Vector3.Cross (xAxis, shoulderDist);
		Vector3 shoulderCol2 = Vector3.Cross (yAxis, shoulderDist);
		Vector3 shoulderCol3 = Vector3.Cross (zAxis, shoulderDist);

		Vector3 elbowCol1 = Vector3.Cross (xAxis, elbowDist);
		Vector3 elbowCol2 = Vector3.Cross (yAxis, elbowDist);
		Vector3 elbowCol3 = Vector3.Cross (zAxis, elbowDist);

		Vector3 endCol1 = Vector3.Cross (xAxis, endDist);
		Vector3 endCol2 = Vector3.Cross (yAxis, endDist);
		Vector3 endCol3 = Vector3.Cross (zAxis, endDist);
		
		jacobian[1,1] = new Complex(System.Convert.ToDouble(shoulderCol1.x), 0);
		jacobian[1,2] = new Complex(System.Convert.ToDouble(shoulderCol2.x), 0);
		jacobian[1,3] = new Complex(System.Convert.ToDouble(shoulderCol3.x), 0);
		jacobian[1,4] = new Complex(System.Convert.ToDouble(elbowCol1.x), 0);
		jacobian[1,5] = new Complex(System.Convert.ToDouble(elbowCol2.x), 0);
		jacobian[1,6] = new Complex(System.Convert.ToDouble(elbowCol3.x), 0);
		jacobian[1,7] = new Complex(System.Convert.ToDouble(endCol1.x), 0);
		jacobian[1,8] = new Complex(System.Convert.ToDouble(endCol2.x), 0);
		jacobian[1,9] = new Complex(System.Convert.ToDouble(endCol3.x), 0);
		jacobian[2,1] = new Complex(System.Convert.ToDouble(shoulderCol1.y), 0);
		jacobian[2,2] = new Complex(System.Convert.ToDouble(shoulderCol2.y), 0);
		jacobian[2,3] = new Complex(System.Convert.ToDouble(shoulderCol3.y), 0);
		jacobian[2,4] = new Complex(System.Convert.ToDouble(elbowCol1.y), 0);
		jacobian[2,5] = new Complex(System.Convert.ToDouble(elbowCol2.y), 0);
		jacobian[2,6] = new Complex(System.Convert.ToDouble(elbowCol3.y), 0);
		jacobian[2,7] = new Complex(System.Convert.ToDouble(endCol1.y), 0);
		jacobian[2,8] = new Complex(System.Convert.ToDouble(endCol2.y), 0);
		jacobian[2,9] = new Complex(System.Convert.ToDouble(endCol3.y), 0);
		jacobian[3,1] = new Complex(System.Convert.ToDouble(shoulderCol1.z), 0);
		jacobian[3,2] = new Complex(System.Convert.ToDouble(shoulderCol2.z), 0);
		jacobian[3,3] = new Complex(System.Convert.ToDouble(shoulderCol3.z), 0);
		jacobian[3,4] = new Complex(System.Convert.ToDouble(elbowCol1.z), 0);
		jacobian[3,5] = new Complex(System.Convert.ToDouble(elbowCol2.z), 0);
		jacobian[3,6] = new Complex(System.Convert.ToDouble(elbowCol3.z), 0);
		jacobian[3,7] = new Complex(System.Convert.ToDouble(endCol1.z), 0);
		jacobian[3,8] = new Complex(System.Convert.ToDouble(endCol2.z), 0);
		jacobian[3,9] = new Complex(System.Convert.ToDouble(endCol3.z), 0);
	}

	// PsuedoInverse = Inverse(Transpose * Jacobian) * Transpose
	void pseudoInverse() {
		
		//Compute transpose of jacobian
		Matrix jacobianTran = jacobian.Transpose();
		
		//compute (Transpose * Jacobian)
		Matrix temp = jacobianTran * jacobian;
		
		//Invert the square matrix
		temp.Inverse();
		
		//Multiply inverse by Transpose to give you Psuedo Inverse
		pseudoInv = temp * jacobianTran;	//NEEDS TO BE GLOBAL
		
		
	}

	//compute change in DOF for Joints.
	// Delta Theta = 6x3Jacobian Inverse * 3x1deltaE
	void computeNewJoints() {
		
		//deltaE is 3x1 of Vector to target from end effector
		Vector3 deltaTranslation = target.transform.position - hand.transform.position;
		
		//Convert to matrix
		Matrix deltaE = new Matrix(3,1);
		deltaE[1,1] = new Complex(System.Convert.ToDouble(deltaTranslation.x));
		deltaE[2,1] = new Complex(System.Convert.ToDouble(deltaTranslation.y));
		deltaE[3,1] = new Complex(System.Convert.ToDouble(deltaTranslation.z));
		
		//Have we reached our destination?
		if (Vector3.Distance (target.transform.position, hand.transform.position) < minDistance) {
			Debug.Log("target distance: " + Vector3.Distance (target.transform.position, hand.transform.position));
			//done = true;
		}
		
		//Final calculation for DOF in joints
		Matrix deltaTheta = pseudoInv * deltaE;
		
		//update joints
		xRot += ((float)deltaTheta[1,1].Re * step);
		yRot += ((float)deltaTheta[2,1].Re * step);
		zRot += ((float)deltaTheta[3,1].Re * step);

		elbowscript.xRot += ((float)deltaTheta[4,1].Re * step);
		elbowscript.yRot += ((float)deltaTheta[5,1].Re * step);
		elbowscript.zRot += ((float)deltaTheta[6,1].Re * step);

		hand.Rotate (((float)deltaTheta [7, 1].Re * step), 
		             ((float)deltaTheta [8, 1].Re * step), 
		             ((float)deltaTheta [9, 1].Re * step),
		             Space.World);
	}

	// Update is called once per frame
	void Update () {

		if(done == false) {
			jacobianCalculation ();
			pseudoInverse();
			computeNewJoints();


			xRot = Mathf.Clamp(xRot, xMin, xMax);
			yRot = Mathf.Clamp(yRot, yMin, yMax);
			zRot = Mathf.Clamp(zRot, zMin, zMax);

			elbowscript.xRot = Mathf.Clamp(elbowscript.xRot, elbowscript.xMin, elbowscript.xMax);
			elbowscript.yRot = Mathf.Clamp(elbowscript.yRot, elbowscript.yMin, elbowscript.yMax);
			elbowscript.zRot = Mathf.Clamp(elbowscript.zRot, elbowscript.zMin, elbowscript.zMax);


			transform.rotation = Quaternion.Euler(xRot,yRot,zRot);
			elbow.transform.rotation = Quaternion.Euler(elbowscript.xRot, elbowscript.yRot, elbowscript.zRot);
		}

	}
}
