using UnityEngine;
using UnityEditor;

[ExecuteInEditMode]
public class ExampleScript : MonoBehaviour
{
    public float value = 7.0f;
    // Debug.Log("That called");
}

// A tiny custom editor for ExampleScript component
[CustomEditor(typeof(ExampleScript))]
public class ExampleEditor : Editor
{

    // public 

    // Custom in-scene UI for when ExampleScript
    // component is selected.
    public void OnSceneGUI()
    {
        // Debug.Log("This called");
        var t = target as ExampleScript;
        var tr = t.transform;
        var pos = tr.position;
        // display an orange disc where the object is
        var color = new Color(1, 0.8f, 0.4f, 1);
        Handles.color = color;
        Handles.DrawWireDisc(pos, tr.up, 1.0f);
        // display object "value" in scene
        GUI.color = color;
        Handles.Label(pos, t.value.ToString("F1"));
    }
    
    void OnEnable()
    {
        // Debug.Log("OnEnable called");
    }

    // Possibly use this to show actual vs estimated values
    // public override void OnInspectorGUI()
    // {
    //     Debug.Log("OnInspectorGUI called");
    //     // serializedObject.Update();
    //     // EditorGUILayout.PropertyField(lookAtPoint);
    //     // serializedObject.ApplyModifiedProperties();
    // }
}
