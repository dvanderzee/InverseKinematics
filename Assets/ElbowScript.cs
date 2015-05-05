using UnityEngine;
using System.Collections;

public class ElbowScript : MonoBehaviour {


	public float xRot;
	public float zRot;
	public float yRot;
	// Min and max rotation for the elbow
	public float xMin, xMax;
	public float zMin, zMax;
	public float yMin, yMax;
	// Rotation Speed
	public float rotSpeed;

	// Use this for initialization
	void Start () {

	}
	
	// Update is called once per frame
	void Update () {
		if(Input.GetKey(KeyCode.Z)) {
			Debug.Log("Rotating Elbow over X Positively");
			xRot += rotSpeed;
			xRot = Mathf.Clamp(xRot, xMin, xMax);
			transform.rotation = Quaternion.Euler(xRot, transform.rotation.y, zRot);

		}

		if(Input.GetKey(KeyCode.X)) {
			Debug.Log("Rotating Elbow over X Negatively");
			xRot -= rotSpeed;
			xRot = Mathf.Clamp(xRot, xMin, xMax);
			transform.rotation = Quaternion.Euler(xRot, transform.rotation.y, zRot);
		}

		if(Input.GetKey(KeyCode.C)) {
			Debug.Log("Rotating Elbow over Z Positively");
			zRot += rotSpeed;
			zRot = Mathf.Clamp(zRot, zMin, zMax);
			transform.rotation = Quaternion.Euler(xRot,transform.rotation.y,zRot);
		}
		
		if(Input.GetKey(KeyCode.V)) {
			Debug.Log("Rotating Elbow over Z Negatively");
			zRot -= rotSpeed;
			zRot = Mathf.Clamp(zRot, zMin, zMax);
			transform.rotation = Quaternion.Euler(xRot,transform.rotation.y,zRot);
		}
		/*
		if(Input.GetKey(KeyCode.B)) {
			Debug.Log("Rotating Elbow over Z Positively");
			yRot += rotSpeed;
			yRot = Mathf.Clamp(yRot, yMin, yMax);
			transform.rotation = Quaternion.Euler(xRot,yRot,zRot);
		}
		
		if(Input.GetKey(KeyCode.N)) {
			Debug.Log("Rotating Elbow over Z Negatively");
			yRot -= rotSpeed;
			yRot = Mathf.Clamp(yRot, yMin, yMax);
			transform.rotation = Quaternion.Euler(xRot,yRot,zRot);
		}
		*/
	}
}
