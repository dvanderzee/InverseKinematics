using UnityEngine;
using System.Collections;

public class ShoulderMove : MonoBehaviour {

	public Transform arm1;

	Vector3 move;
	Vector3 axis;
	ConfigurableJoint myJoint;
	float rotateSpeed;

	// Rotation floats
	float yRot = 0;
	float minRot = -45f;
	float maxRot = 45f;

	// Use this for initialization
	void Start () {
		move = new Vector3(1,1,1);
		axis = new Vector3 (10f, 0, 0);
		rotateSpeed = 5f;

		yRot = 3;
		yRot = Mathf.Clamp (yRot, minRot, maxRot);
	}
	
	// Update is called once per frame
	void Update () {

		gameObject.transform.eulerAngles = new Vector3 (gameObject.transform.eulerAngles.x, yRot, gameObject.transform.eulerAngles.z);
		gameObject.transform.Rotate (yRot, 0, 0);

		// Rotation Controls
		if(Input.GetKey(KeyCode.A)) {
			Debug.Log("Rotating Shoulder");
		}
	}
}
