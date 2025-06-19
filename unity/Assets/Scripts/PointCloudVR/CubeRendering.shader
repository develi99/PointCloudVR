Shader "Custom/InstancedIndirectColorShader"
{
    Properties
    {
        _BaseColor ("Base Color", Color) = (0.5, 0.1,0.2,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            // Standard Lighting abschalten
            Cull Off
            ZWrite On
            ZTest LEqual
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5

            #include "UnityCG.cginc"

            // Structured Buffers
            StructuredBuffer<float4x4> matrixBuffer;
            Texture2D<float4> _ColorTex;
            int _Width;

            float4 _BaseColor;

            struct appdata
            {
                float3 vertex : POSITION;
                float3 normal : NORMAL;
                uint instanceID : SV_InstanceID;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
            };

            v2f vert(appdata v)
            {
                v2f o;

                float4x4 model = matrixBuffer[v.instanceID];
                float4 worldPos = mul(model, float4(v.vertex, 1.0));
                o.pos = UnityObjectToClipPos(worldPos);


                uint x = v.instanceID % _Width;
                uint y = v.instanceID / _Width;

                float4 color = _ColorTex.Load(int3(x, y, 0));
                o.color = color;

                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
