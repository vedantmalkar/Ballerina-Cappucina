Onshape is a tool used to create visual and virtual blue prints for your projects.

An onshape project can be separated into different tabs for simplicity and tabs can be combined into a folder 
# ![[Screenshot from 2025-06-26 16-18-20.png]]

Exploration (full list in help menu) :
- in and out using centre wheel 
- rotate using right click and drag
- wheel button to pan
- f to centre model 
- shift + 7 for eye view 
- bottom right contains measurements

Assemblies are where you define the product hierarchy and motion between parts

Mates are how you add relationships between parts in assembly, each mate defines DOF (total 6)

Onshape Mates take advantage of Mate connectors, which are fully defined 3D coordinate systems

Onshape contains a library of fasteners and hardware called Standard content

To hide visible mate connectors use shortcut K

The Bill of Materials (BOM) is accessed as a tab on the right side of an Assembly tab:

 ![[Screenshot from 2025-06-26 16-47-39.png]]


Every action in Onshape is recorded in Versions and history
    
Branch to create variant Workspace or test design changes
    
Merge to carry over actions from one branch to another

Folders are used to group tabs within a Document.

Unpacking folders deletes the folder, but not its contents.

Folders can be nested up to four levels deep.

To start working select sketch and then select a plane, when done you may click on the check mark

Create construction geometry to begin with or change existing geometry to construction.
Use the Q key to easily toggle construction, great for when you are in the middle of sketching geometry.
Create construction ![](https://lh7-us.googleusercontent.com/n6qM0JGYRlo0drFHJkYU4h02IKUJH99cHl8bze3JwKoifrhPSuhSnIbm5B4nKkviTT9q0kq_tycQDFXLLOR7-FTKBbX875TqjM2jhINEASIs2xHuAdTtMm_OSi9W24SFfs1RfqPpgkR0InA6kW43QXw "construction") lines ![](https://lh7-us.googleusercontent.com/2Ua1dwT3x41tYfWYeekPAJSle-w8dXAUZGodsAlKpRV_gQRFnz29_Q4HbpPsBeb2YBn4yRZFIwR3aHFp6VWxZRCMG_-Kjul-lD6qlkbe85jcVXgy7Yfo8XL5S0sWbaLMzIRKHAEwJ-vM_55t--WgheM "line") and arcs ![](https://lh7-us.googleusercontent.com/OBwtTF9wrU1CuRbWCRIAMac7TiofrPp5R2QX9Gk8Bz7aOSDm8hsM_4Xg6TIyAuX-x9Qc5Rfk3RZ4Uh4PsMzBqMDmCmchIm1A_84ucu8uGanMvnfWylJajySqqYn6eQZ9RopA9Ud6U5XwP2xQxQJmCpA "3 point arc") to define the centerline of a slot.
Polygons are created with an inscribed ![](https://lh7-us.googleusercontent.com/lYNJyg8l7RLy9xgNdMN4rur1CU6b8TkUqbGTtiP-0wGW2qAMYuC12rKPZi0jrp2hCQs9RPKU25Uf_5dF7ebMBQ9NrCWfXH6G4IQvjEuQPBqaM6_rhbaDXqQ1vz8HjN88SaIsf6nFddeVYEkCW5Wx2zY "inscribed polygon") or circumscribed ![](https://lh7-us.googleusercontent.com/J21yBTb6gDtm-NKbI-8pu5QMnfsSA7qAskHhjcEdpW1XxJGx-hC9RoW01PKc_INLrg_RNqzdv4yDBgoD5e5ougwLtV9Zs3gfhYAyJpZjUTxZsf2DE3ytjbCjKkaK4ASQ90CLcTuWoZtGbKGftto_SXk "circumscribed polygon") construction circle.
Use Sketch points to define the location of holes, create planes, curves, or other types of geometry.

Use inferencing to automatically apply constraints as you sketch
Hover over an entity to "wake up" its inferences
Click the checkbox in the sketch dialog to show all constraints
Hold shift to temporarily remove constraints 

- Add dimensions while you sketch or add them afterwards
- The first dimension you apply scales the entire sketch
- You can add dimension leader lines to the circumference of circles

Drawing completed using multiple Onshape accessibility 

![[Screenshot from 2025-06-29 12-09-06.png]]


- Onshape's use of sketches is more flexible than more traditional 3D CAD systems.
- Create features with a whole sketch, or portions of the sketch geometry.
- Sweep and Revolve inherit the same flexible behavior.

EXTRUDE:

![[Screenshot from 2025-06-29 12-11-41.png]]

- The Extrude feature extends a sketch along a specified direction to create depth.
- Select from multiple end conditions.
- Extrude solid parts, surfaces, or thin solids.

- The Extrude feature contains options to create a new part, add to or remove from existing parts, or keep the feature's intersection with existing parts.
- Other features with Boolean options include Revolve, Sweep, Loft, Thicken, Patterns, and Mirror.2
- In addition to the Boolean options within these Features, Onshape also provides a Boolean feature.

Drawing 2:
![[Screenshot from 2025-06-29 12-41-07.png]]

After Extrude:
![[Screenshot from 2025-06-29 12-46-48.png]]


- The Revolve feature revolves a selection about an axis.
- A revolve creates a solid part, surface, or thin solid.
- The Sketch regions to revolve must create a closed profile for a Solid revolve.


![[Screenshot from 2025-06-29 12-57-53.png]]

![[Screenshot from 2025-06-29 13-10-05.png]]

![[Screenshot from 2025-06-29 13-10-33.png]]


![[Screenshot from 2025-06-29 13-25-15 1.png]]
- Applying a material to parts allows you to track material choices for your design.
- Customize your material library.
- Mass and section properties provide additional insight based on your material choices.

- Features often depend on features higher in the Feature list.
- To show both parent and child features, right-click on a feature in the Feature list and select Show dependencies.
- To modify the order, drag and drop features within the Feature list.

- Every Part Studio within Onshape contains three planes: the Top, Front, and Right.
- The Plane feature provides numerous methods to define a reference plane.
- Use reference planes as sketch planes, to specify direction, or to mirror parts, features or faces.
- 
![[Screenshot from 2025-06-29 19-20-38.png]]

- Part design is an iterative process, often requiring multiple versions and revisions to arrive at the final product.
- The slider at the bottom of the Feature dialog offers insight into the model before and after generating the feature.
- The Final button previews the resulting Part Studio once all features have been regenerated.

- The Fillet feature rounds exterior and interior corners, while the Chamfer feature breaks sharp edges with a bevel.
- The Fillet dialog contains two tabs at the top: Edge fillet and Full round fillet.
- Define Chamfers by one of two different measurement types: Offset or Tangent.

- The Hole feature places a hole at any selected sketch point, aligning the center of the hole to the point.
- You may also utilize implicit and explicit Mate connectors to drive the location of holes.
- A Hole callout annotation can only be added to a hole created with the Hole feature.

- The Shell feature generates thin-walled parts, which is common in plastic part design.
- The Shell operation is an applied feature and does not require a pre-defined sketch.
- For hollow parts with no outside openings, select the hollow checkbox, then select the part.

![[Screenshot from 2025-06-29 20-11-37.png]]

![[Screenshot from 2025-06-29 20-16-11.png]]

- Drive the parts of an assembly based on the location or dimensions of other parts using a top-down modeling approach.
- In a Part Studio, a single sketch or feature can drive the geometry of several parts.
- You can apply features like fillets to multiple parts at once.
- when needed you can split a part into multiple sections and join them later on 

- The sweep feature sweeps a profile along a path to create a swept solid, surface, or thin solid.
- The path may consist of one or more sketch entities, an entire sketch, a curve, or model edges.
- The profile can be closed or open. Use a closed profile sketch or a part face to create a solid sweep.

- The loft feature defines complex part geometry by utilizing sketch profiles and 3D curves.
- Select the loft profiles in sequential order as the feature should propagate.
- Under End conditions, set the Start profile condition and the End profile condition.

![[Screenshot from 2025-06-29 20-45-35.png]]

![[Screenshot from 2025-06-29 20-46-06.png]]

![[Screenshot from 2025-06-29 20-53-16.png]]

![[Screenshot from 2025-06-29 20-56-12.png]]

![[Screenshot from 2025-06-29 21-07-56.png]]

- Patterns efficiently replicate parts, individual features, or faces.
- Part patterns create new parts, add or remove material from geometry, or preserve intersecting geometry.
- Feature pattern replicates one or more features selected from the Features list.
- Face patterns replicate selected faces.

- The Circular pattern feature replicates selected parts, features, or faces about a center axis.
- Valid selections for the pattern axis are a circular edge, a cylindrical face, or even a circle or construction circle in a sketch.
- You can skip specific pattern instances by checking the Skip instances option.

- The Mirror feature replicates parts, surfaces, faces, or features across a mirror plane.
- Consider modeling half of a part and use a Part mirror with the Add tab to mirror the symmetric geometry.
- Use Create selection to more easily select the subset of faces to be mirrored.
![[Screenshot from 2025-06-29 22-14-30.png]]

After Fillet:

![[Screenshot from 2025-06-29 22-18-27.png]]

After repeating pattern:

![[Screenshot from 2025-06-29 22-21-21.png]]

After Hole 1:

![[Screenshot from 2025-06-29 22-23-28.png]]

After 2nd hole and repeated:
![[Screenshot from 2025-06-29 22-30-47.png]]

Adding rectangles:

![[Screenshot from 2025-06-29 22-35-05.png]]

- With Onshape, you can design multiple parts in the same Part Studio.
- Parts within the same Part Studio can share features and dimensions.
- Each part in a Part Studio is independent, with its own properties.