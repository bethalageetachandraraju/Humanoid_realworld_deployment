import mujoco
from mujoco import viewer

model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
""")
data = mujoco.MjData(model)

with viewer.launch_passive(model, data) as v:
    for _ in range(1000):
        mujoco.mj_step(model, data)
        v.sync()
