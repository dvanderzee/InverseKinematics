using UnityEngine;
using System.Collections;

public class ElbowScript : MonoBehaviour {

	Vector3 move;
	Vector3 axis;
	ConfigurableJoint myJoint;
	float rotateSpeed;

	// Use this for initialization
	void Start () {
		move = new Vector3(1,1,1);
		axis = new Vector3 (10f, 0, 0);
		rotateSpeed = 5f;
	}
	
	// Update is called once per frame
	void Update () {
		if(Input.GetKey(KeyCode.D)) {
			gameObject.transform.Rotate(Vector3.up * 5, Space.World);
			Debug.Log("Rotating Elbow");
		}
	}
}
