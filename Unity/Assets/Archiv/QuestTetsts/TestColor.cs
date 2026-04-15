using UnityEngine;

[RequireComponent(typeof(Renderer))]
public class SolidColorTexture : MonoBehaviour
{
    public int textureSize = 256;
    public Color fillColor = Color.blue;

    void Start()
    {
        // 1. Neue Textur erzeugen
        Texture2D texture = new Texture2D(textureSize, textureSize, TextureFormat.RGBA32, false);

        // 2. Alle Pixel mit konstanter Farbe füllen
        Color[] pixels = new Color[textureSize * textureSize];
        for (int i = 0; i < pixels.Length; i++)
            pixels[i] = fillColor;

        texture.SetPixels(pixels);
        texture.Apply();

        // 3. Textur auf Material setzen
        Renderer renderer = GetComponent<Renderer>();
        Material mat = renderer.material;
        mat.shader = Shader.Find("Unlit/Texture");
        mat.mainTexture = texture;
    }
}
