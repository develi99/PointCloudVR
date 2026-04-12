Shader "Custom/CubeRenderingPoints"
{
    Properties
    {
        _BaseColor("Base Color", Color) = (1,1,1,1)
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            ZWrite On
            ZTest LEqual
            Cull Off

            CGPROGRAM
            #pragma target 4.5
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"

            StructuredBuffer<float4x4> matrixBuffer;
            StructuredBuffer<float3> colorBuffer;

            float4 _BaseColor;

            struct appdata
            {
                float3 vertex : POSITION;
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
                o.pos = mul(UNITY_MATRIX_VP, worldPos);

                float3 c = colorBuffer[v.instanceID];
                o.color = float4(c, 1.0);
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
