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

	// Speed at which objects are manually rotated
	public float rotSpeed;

	// Use this for initialization
	void Start () {

	}
	
	// Update is called once per frame
	void Update () {
	
		// Manual Rotation Controls
		if(Input.GetKey(KeyCode.A)) {
			Debug.Log("Rotating Shoulder over X Positively");
			xRot += rotSpeed;
		}

		if(Input.GetKey(KeyCode.S)) {
			Debug.Log("Rotating Shoulder over Y Positively");
			yRot += rotSpeed;
		}

		if(Input.GetKey(KeyCode.D)) {
			Debug.Log("Rotating Shoulder over Z Positively");
			zRot += rotSpeed;
		}

		if(Input.GetKey(KeyCode.Q)) {
			Debug.Log("Rotating Shoulder over X Negatively");
			xRot -= rotSpeed;
		}
		
		if(Input.GetKey(KeyCode.W)) {
			Debug.Log("Rotating Shoulder over Y Negatively");
			yRot -= rotSpeed;
		}
		
		if(Input.GetKey(KeyCode.E)) {
			Debug.Log("Rotating Shoulder over Z Negatively");
			zRot -= rotSpeed;
		}

		// Clamping controls
		xRot = Mathf.Clamp(xRot, xMin, xMax);
		yRot = Mathf.Clamp(yRot, yMin, yMax);
		zRot = Mathf.Clamp(zRot, zMin, zMax);

		// Actual rotation command;
		transform.localEulerAngles = new Vector3(xRot, yRot, zRot);
	}
}
