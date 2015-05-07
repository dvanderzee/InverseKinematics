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

	// Values to be used in computing Jacobian Inverse
	public float phi;	// Angle between line from origin to target and x axis
	public float theta;	// Angle between line from origin to target and z-axis
	public float rho;	// Distance between origin and target

	// Matrices to hold our Jacobian Values and it's Adjacency Matrix
	float[,] jacobian = new float[3, 2];
	float[,] adj = new float[3, 2];
	float[,] inv = new float[3, 2];

	// Speed at which objects are manually rotated
	public float rotSpeed;

	// Determinant used in inverting the matrix
	private float determinant;

	public GameObject elbowObject;
	public float elbowX, elbowY, elbowZ;

	// Use this for initialization
	void Start () {

		// Vector from Hand to Target
		Vector3 handToTarget = target.position - hand.position;

		// Vector3.right is the x-axis
		phi = Vector3.Angle (Vector3.right, handToTarget);

		// Vector3.forward is the z axis
		theta = Vector3.Angle (Vector3.forward, handToTarget);

		// Again, distance between hand and target
		rho = Vector3.Distance (hand.position, target.position);

		elbowObject = GameObject.Find ("Elbow");
		ElbowScript elbowscript = elbowObject.GetComponent<ElbowScript> ();
		elbowX = elbowscript.xRot;
		elbowY = elbowscript.yRot;
		elbowZ = elbowscript.zRot;

	}

	// (Axis of Revolution) X (Vector from joint to end effector)
	void jacobianCalculation() {
		Vector3 shoulderAxis;		// Axis of Revolution for Shoulder
		Vector3 elbowAxis;			// Axis of Revolution for Elbow

		Vector3 shoulderRot = new Vector3 (xRot, yRot, zRot);		// Current Rotation Vector for Shoulder
		Vector3 elbowRot = new Vector3 (elbowX, elbowY, elbowZ);	// Current Rotation Vector for Elbow

		float shoulderRotMag = 	// Magnitude of the Shoulder Rotation
			Mathf.Sqrt (Mathf.Pow (xRot, xRot) + Mathf.Pow (yRot, yRot) + Mathf.Pow (zRot, zRot));
		float elbowRotMag = 	// Magnitude of the Elbow Rotation
			Mathf.Sqrt (Mathf.Pow (elbowX, elbowX) + Mathf.Pow (elbowY, elbowY) + Mathf.Pow (elbowZ, elbowZ));

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

	}

	void pseudoInverse() {
		float[,] transpose = new float[2, 3];

		transpose [0, 0] = jacobian [0, 0];
		transpose [0, 1] = jacobian [1, 0];
		transpose [0, 2] = jacobian [2, 0];
		transpose [1, 0] = jacobian [0, 1];
		transpose [1, 1] = jacobian [1, 1];
		transpose [1, 2] = jacobian [2, 1];
	
	}

	void jacobianInverse(){
		/*
		// Some assignments to make readibility easier
		float a11, a12, a13, a21, a22, a23, a31, a32, a33;
		a11 = jacobian [0, 0];
		a12 = jacobian [0, 1];
		a13 = jacobian [0, 2];
		a21 = jacobian [1, 0];
		a22 = jacobian [1, 1];
		a23 = jacobian [1, 2];
		a31 = jacobian [2, 0];
		a32 = jacobian [2, 1];
		a33 = jacobian [2, 2];

		// We first find the determinant of the Jacobian
		determinant = (a11 * ((a33 * a22) - (a32 * a23)))
			- (a21 * ((a33 * a12) - (a32 * a13)))
			+ (a31 * ((a23 * a12) - (a22 * a13)));

		// We then calculate the adjacency matrix
		adj [0, 0] = ((a33 * a22) - (a32 * a23));
		adj [0, 1] = -((a33 * a12) - (a32 * a13));
		adj [0, 2] = ((a23 * a12) - (a22 * a13));
		adj [1, 0] = -((a33 * a21) - (a31 * a23));
		adj [1, 1] = ((a33 * a11) - (a31 * a13));
		adj [1, 2] = -((a23 * a11) - (a21 * a13));
		adj [2, 0] = ((a32 * a21) - (a31 * a22));
		adj [2, 1] = -((a32 * a11) - (a31 * a31));
		adj [2, 2] = ((a22 * a11) - (a21 * a12));

		// Now we calculate each cell of the inverse
		inv [0, 0] = adj [0, 0] / determinant;
		inv [0, 1] = adj [0, 0] / determinant;
		inv [0, 2] = adj [0, 0] / determinant;
		inv [1, 0] = adj [0, 0] / determinant;
		inv [1, 1] = adj [0, 0] / determinant;
		inv [1, 2] = adj [0, 0] / determinant;
		inv [2, 0] = adj [0, 0] / determinant;
		inv [2, 1] = adj [0, 0] / determinant;
		inv [2, 2] = adj [0, 0] / determinant;
		*/
	}
	
	// Update is called once per frame
	void Update () {
	
		// Manual Rotation Controls
		if(Input.GetKey(KeyCode.A)) {
			Debug.Log("Rotating Shoulder over X Positively");
			Debug.Log(xRot);
			xRot += rotSpeed;
			xRot = Mathf.Clamp(xRot, xMin, xMax);
			transform.rotation = Quaternion.Euler(xRot, yRot, zRot);
		}

		if(Input.GetKey(KeyCode.S)) {
			Debug.Log("Rotating Shoulder over Y Positively");
			yRot += rotSpeed;
			yRot = Mathf.Clamp(yRot, yMin, yMax);
			transform.rotation = Quaternion.Euler(xRot, yRot, zRot);
		}

		if(Input.GetKey(KeyCode.D)) {
			Debug.Log("Rotating Shoulder over Z Positively");
			zRot += rotSpeed;
			zRot = Mathf.Clamp(zRot, zMin, zMax);
			transform.rotation = Quaternion.Euler(xRot, yRot, zRot);
		}

		if(Input.GetKey(KeyCode.Q)) {
			Debug.Log("Rotating Shoulder over X Negatively");
			xRot -= rotSpeed;
			xRot = Mathf.Clamp(xRot, xMin, xMax);
			transform.rotation = Quaternion.Euler(xRot, yRot, zRot);
		}
		
		if(Input.GetKey(KeyCode.W)) {
			Debug.Log("Rotating Shoulder over Y Negatively");
			yRot -= rotSpeed;
			yRot = Mathf.Clamp(yRot, yMin, yMax);
			transform.rotation = Quaternion.Euler(xRot, yRot, zRot);
		}
		
		if(Input.GetKey(KeyCode.E)) {
			Debug.Log("Rotating Shoulder over Z Negatively");
			zRot -= rotSpeed;
			zRot = Mathf.Clamp(zRot, zMin, zMax);
			transform.rotation = Quaternion.Euler(xRot, yRot, zRot);
		}

	}
}
