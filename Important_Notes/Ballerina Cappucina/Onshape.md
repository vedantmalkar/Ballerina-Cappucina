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

- Create a Master Sketch to drive multiple features and parts within the Part Studio.
- Master Sketches define key geometry and capture the spatial relationships between parts.
- Easily update Part Studios by editing Master Sketches.


- Assemblies are used to define motion between parts, instance parts, and assemble parts.
- One document can contain several assembly elements.
- Use the triad manipulator for more control when moving components.

- Always fix ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027987/nardff0un1g4vm7bvjvl.png) a single part to the assembly origin.
- To align a part to the assembly origin, use the green checkmark on the insert dialog or the triad manipulator options.
- Aligning a part to the origin does _not_ automatically fix ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027987/nardff0un1g4vm7bvjvl.png) the part.

- Mate connectors define a local coordinate system with X, Y, and Z axes.
- Each mate requires two mate connectors.
- Mate connectors are attached to automatically generated mate connection points.
- It is only necessary to have one mate between any two parts.
- mates can be defined with limits, which limits its movements

- The Fastened ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027732/w28mpmlh4ddvb0joy53z.png) mate eliminates all motion.
- The Revolute ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027764/l7kvb9nz9tiglwvdid9p.png), and Slider ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027756/asf8qk5trv6b48cxz41k.png), mates allow one degree of freedom with respect to the primary axis.
- The Cylindrical ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027744/gdweuk8ncsosvylsib0t.png) mate allows two degrees of freedom with respect to the primary axis.
- Limits establish a minimum and maximum range of motion.

- The Pin slot ![](https://d36ai2hkxl16us.cloudfront.net/course-uploads/6e557ed6-d03d-4c48-9492-4d18d145d7a1/ihch6nzg2yn7-pinslot-mdpi.png) mate allows rotation about the primary axis and translation about the X axis of the slot.
- The Parallel ![](https://d36ai2hkxl16us.cloudfront.net/course-uploads/6e557ed6-d03d-4c48-9492-4d18d145d7a1/iiofrt421xgx-parallel-mdpi.png) mate allows four degrees of freedom; rotation about the primary axis and translation in X, Y, and Z axes.
- The Ball ![](https://d36ai2hkxl16us.cloudfront.net/course-uploads/6e557ed6-d03d-4c48-9492-4d18d145d7a1/mjjnkx7uapyq-ball-mdpi.png) mate allows rotation about the X, Y, and Z axes.
- The Planar ![](https://d36ai2hkxl16us.cloudfront.net/course-uploads/6e557ed6-d03d-4c48-9492-4d18d145d7a1/h1n1syrto8po-planar-mdpi.png) mate allows translation in the X and Y axes and rotation about the primary axis.

- The Tangent mate is unique and requires a face, edge, or vertex selection from each part to define the tangent relationship.
- Sketch entities and surfaces are valid choices to be tangent.
- The Tangent propagation option ensures the Tangent constraint is propagated to adjacent tangent faces.

- It is best practice to Fix ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027888/qbuk7rptzkbk7y4c88up.png) only one part in an assembly.
- Use the group ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1489027876/fu3tcwars7nwjykiotgh.png) command for parts that should not move relative to one another.
- Take advantage of using groups wherever possible.

when creating an assembly first we add a part and then we fix it either to origin or not 


![[Screenshot from 2025-06-30 18-03-03.png]]


- Explicit mate connectors should be created whenever mate connection points are not sufficient.
- An explicit mate connector that is created in a Part Studio, must be owned by a part.
- Inserting parts into an assembly that own explicit mate connectors, also inserts the mate connectors into the assembly.

- Mate connectors created within a Mate are called implicit Mate connectors and are located under the Mate in the Mate Features list.
- The associative part highlights when hovering over an implicit Mate connector.
- Edit Mate connectors by realigning to geometry entities, moving, flipping the primary alignment, or reorienting the secondary alignment.

Creating Explicit Mate Connectors Exerciser:
![[Screenshot from 2025-06-30 18-24-29.png]]

- Use the triad manipulator for precise control over assembly motion.
- The triad manipulator can be repositioned by snapping its center to mate connection points.
- Animate allows you to view the assembly’s motion through one degree of freedom.
- Notice the visual indicators in the Instance list that inform you of parts that can move.

- Use subassemblies wherever possible.
- Parts with motion in subassemblies will still have motion in higher-level assemblies.
- A part that is fixed in a subassembly will _not_ be fixed in higher-level assemblies.

- Isolate shows all other parts muted in color and unavailable for selection.
- You can hide and show parts, mates, and mate connectors using shortcut keys **Y**, **Shift+Y**, **J**, and **K** respectively.
- The “Switch to” command allows you to quickly flip from an assembly to a part’s Part Studio.




![[Screenshot from 2025-06-30 20-28-52.png]]

- Use the Standard Content options in the Insert dialog to quickly insert standard components into any assembly.
- Auto-size selects the best size fastener for any hole or shaft.
- Use Insert closest and furthest from selection to define a stack order as you insert parts.

- Organize assembly instances with folders.
- Hide/show and suppress/unsuppress folders without having to make a large selection set.
- Delete folder removes the folder and its contents.
- Unpack folder removes the folder leaving the contents in the Instance list.
- Right-click on a mate and choose Add selection to folder to create mate folders.

- Drawings can be created through the Create element menu, or by right-clicking a part, Part Studio, or Assembly and selecting Create Drawing.
- Onshape provides standard templates and the ability to create custom templates.
- Drawings can contain multiple sheets.
- Each drawing tab, drawing sheet, and view has its own set of properties.

- Views can be added to a drawing sheet by projecting them from an existing view or by using the Insert view ![](https://d36ai2hkxl16us.cloudfront.net/thoughtindustries/image/upload/v1490559310/y3l2h7fdak97xrx4bm0p.png) command.
- Orthographic views are aligned by default, but the alignment can be suppressed.
- Hidden lines and shaded views can be shown through the right-click menu.
- Click and drag views to copy them with the Alt key on PC and the Option key on Mac.

- None turns off view simplification entirely.
- Automatic is the default view simplification allowing Onshape to find the best view simplification setting based on the geometry of the assembly.
- Absolute requires the user to enter a number in length units to Indicate that features smaller than the clue are simplified within the view.
- Ratio to studio requires the user to enter a percentage of the size of the Part Studio or Assembly.
- Ratio to part requires the user to enter a percentage of the size of the part below which the feature is simplified within the view.

- Auxiliary views project from an edge showing a face in true size and true shape.
- Break views break long thin parts to fit on a drawing sheet.

- Broken Out Section views showcase an inner segment of a model view.
- Crop views highlight a portion of a model.

- Use assembly Exploded views on drawings to showcase how a design is assembled and easily apply BOM balloon callouts.
- Display states are reusable visual states of an assembly created by hiding components and mates within an assembly which can be used on a drawing to better dimension or show a portion of a design.
- Hide/show parts in a view on the fly with the right-click menu.

- There are several dimension tools in the drawing toolbar to add specific dimension types based on the entities selected.
- The letter **D** is the keyboard shortcut to activate the dimension tool.
- Grip points of dimension leaders allow dimensions to be adjusted.

- Holes must be created using the Hole feature to use the Hole callout tool in a Drawing.
- Multiple holes can be indicated by one callout with multiple leaders.
- Hole callouts offer a wide range of customizations to communicate the design intent and match the drawing style.
- Hole callouts update to reflect changes made to the Part Studio when the Update from this workspace tool is used.

- Ordinate dimensions allow a set of dimensions to be created from one datum point.
- Dimensions can be added or removed from ordinate dimension groups.
- Individual ordinate dimensions can be repositioned, or the entire ordinate dimension group can be repositioned at once.

- Tolerances for specific dimensions can be set separately from the drawing’s properties by using the dimension panel.
- Symbols and notes can be added to a dimension using the text fields in the dimension panel.
- The precision of a dimension’s nominal and tolerance values can be specified in the dimension panel.

![[Screenshot from 2025-06-30 22-55-30.png]]


- Centerlines and Centermarks are added to views automatically on view placement and can be hidden from the view's right-click menu.
- Centerlines can be added manually by selecting two snap points of a view or two lines of a view.
- Bolt circle centerlines are added manually by selecting the centers of three holes in the pattern or the center point and a point at the perimeter.
- Virtual sharps are displayed as edge extensions or centermarks.


- Choose between the Follow assembly template and available templates created by a company administrator.
- A flattened table displays all the parts.
- A structured multi-level table displays all subassemblies and the parts of each.
- A structured top-level table only displays the top-level parts and subassemblies.


- Callouts can reference Bill of Material table properties or part properties.
- The middle field of the Callout dialog is what displays inside the callout, the other fields display to the top, bottom, left, and right of the callout.
- Right-click to edit a Callout after it is created.


- Create generic tables by defining the number of rows, columns, and headers needed.
- Use the Cell toolbar to merge cells, define row and column widths and heights, and format the cells.
- Insert sheet or drawing reference properties into cell fields to display properties in a table.
- Resize a cell with precision by right-clicking the cell and choosing Resize.


- Hole tables map Hole feature positions using X and Y locations for a view or multiple views of a part and list the holes specifications.
- You may choose to exclude certain holes from the Hole table.
- Customize the X and Y axis for unique parts hole locations.


![[Screenshot from 2025-07-01 03-12-49.png]]


![[Screenshot from 2025-07-01 03-30-32 1.png]]

- To export a drawing right-click on the tab and choose Export…
- You can export a drawing to a DXF, DWG, DWT, or PDF file type.
- If you have an integrated service like Dropbox or Google Drive connected with Onshape, the option Store file in the integrated service becomes available.
- A PDF stored in the document as a tab can be viewed by multiple users.




