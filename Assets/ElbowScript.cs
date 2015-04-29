using UnityEngine;
using System.Collections;

public class ElbowScript : MonoBehaviour {

	// The elbow has only one degree of freedom
	// so only one rotation axis is needed
	public float yRot;

	// Min and max rotation for the elbow
	public float yMin, yMax;

	// Rotation Speed
	public float rotSpeed;

	// Use this for initialization
	void Start () {

	}
	
	// Update is called once per frame
	void Update () {
		if(Input.GetKey(KeyCode.Z)) {
			Debug.Log("Rotating Elbow over Y Positively");
			yRot += rotSpeed;
		}

		if(Input.GetKey(KeyCode.X)) {
			Debug.Log("Rotating Elbow over Y Negatively");
			yRot -= rotSpeed;
		}

		yRot = Mathf.Clamp(yRot, yMin, yMax);
		gameObject.transform.localEulerAngles = new Vector3(transform.localEulerAngles.x, yRot, transform.localEulerAngles.z);
	}
}
