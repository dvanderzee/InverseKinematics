using UnityEngine;
using System.Collections;

public class ElbowScript : MonoBehaviour {

	// The elbow has only one degree of freedom
	// so only one rotation axis is needed
	public float xRot;
	public float zRot;
	// Min and max rotation for the elbow
	public float xMin, xMax;
	public float zMin, zMax;
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
		}

		if(Input.GetKey(KeyCode.X)) {
			Debug.Log("Rotating Elbow over X Negatively");
			xRot -= rotSpeed;
		}

		if(Input.GetKey(KeyCode.C)) {
			Debug.Log("Rotating Elbow over Z Positively");
			zRot += rotSpeed;
		}
		
		if(Input.GetKey(KeyCode.V)) {
			Debug.Log("Rotating Elbow over Z Negatively");
			zRot -= rotSpeed;
		}
		xRot = Mathf.Clamp(xRot, xMin, xMax);
		zRot = Mathf.Clamp(zRot, zMin, zMax);
		gameObject.transform.localEulerAngles = new Vector3(xRot, transform.eulerAngles.y, zRot);
	}
}
