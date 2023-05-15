using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoundingBox : MonoBehaviour
{

    public bool DrawAABB = true;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log(System.String.Format("Start() called on '{0}'", gameObject.name));
    }

    // Update is called once per frame
    void Update()
    {
    	// Debug.Log("Update() called");   
    }
    
    // public void OnDrawGizmosSelected()
    // {
    //     var r = GetComponent<Renderer>();
    //     if (r == null)
    //         return;
    //     var bounds = r.bounds;
    //     Gizmos.matrix = Matrix4x4.identity;
    //     Gizmos.color = Color.blue;
    //     // Gizmos.DrawWireCube(bounds.center, bounds.extents * 2);
    //     Gizmos.DrawWireCube(bounds.center, new Vector3(1, 1, 1));
    // }
    
    public void OnDrawGizmos()
    {
        var r = GetComponent<Renderer>();
        if (r == null)
            return;
        var bounds = r.bounds;
        Gizmos.matrix = Matrix4x4.identity;
        Gizmos.color = Color.red;
        Gizmos.DrawWireCube(bounds.center, bounds.extents * 2);
        // DrawWireCube(Vector3 center, Vector3 size) size = x,y,z size
        // Gizmos.DrawWireCube(bounds.center, new Vector3(1,1,1));
    }

    static Mesh CreateAABB(GameObject gameObject) {
        // Mesh mesh = ((MeshFilter)gameObject.GetComponent("MeshFilter")).mesh;
        // int xMin = yMin = zMin = Integer.MinValue;
        // int xMax = yMax = zMax = Integer.MaxValue;
        Renderer r = gameObject.GetComponent<Renderer>();
        if (r = null) {
            Debug.Log(string.Format("No renderer found for '{0}'", gameObject.name));
            return null;
        }
        
        return null;
    }
}
