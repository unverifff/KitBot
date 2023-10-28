%% Loads in the Kitchen Environment.
% Call in main script with Environment();

classdef Environment 
    methods
        function self = Environment()
            self.setupKitchen();
        end

        function setupKitchen(self)
            self.fig();
            surf([-2, -2; 2, 2], [-2, 2; -2, 2], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([-2, -2; 2, 2], [-2, -6; -2, -6], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([-2, -2; 2, 2], [6, 2; 6, 2], [0, 0.01 ; 0, 0.01] ...
                , 'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([-2, -2; -6, -6], [6, 2; 6, 2], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([-2, -2; -6, -6] , [-2, 2; -2, 2], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([-2, -2; -6, -6], [-2, -6; -2, -6], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([2, 2; 6, 6], [6, 2; 6, 2], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([2, 2; 6, 6], [-2, 2; -2, 2], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
            surf([2, 2; 6, 6], [-2, -6; -2, -6], [0, 0.01 ; 0, 0.01] ...
                ,'CData',imread('Floor.jpg'), 'FaceColor','texturemap');
        
            % Insert Kitchen Island
            file = "Kitchen Island.ply";
            transform = transl(0.5, 0.08, 0.3) * trotx(0) * troty(0) * trotz(deg2rad(0));
            self.insertObj(file, transform);
        
            % Insert Kitchen Benches
            file = "Kitchen Furniture.ply";
            transform = transl(0.45, -2, 0.7) * trotx(0) * troty(0) * trotz(deg2rad(180));
            self.insertObj(file, transform);
        
            % Insert Dishwasher & Fridge
            file = "Fridge Dishwasher.ply";
            transform = transl(-1.75, -1.1, 0) * trotx(0) * troty(0) * trotz(deg2rad(90));
            self.insertObj(file, transform);
        
            % Insert Kitchen Wall
            PlaceObject('Wall.ply', [-2, -2.25, 0]);
        
            % Insert Rails
            PlaceObject('Wall Railing.ply', [-0.5, -2.25, 1.35]);
            PlaceObject('Wall Railing.ply', [0.5, -2.25, 1.35]);
        
            % Insert Wall Hooks
            PlaceObject('Wall hook.ply', [1.75, -2.25, 1.35]);
            PlaceObject('Wall hook.ply', [2, -2.25, 1.35]);
            PlaceObject('Wall hook.ply', [2.25, -2.25, 1.35]);
        
            % Insert e-stop
            PlaceObject('Emergency stop.ply', [-2.05, 0, 1.2]);
        
            % Insert Shelf
            file = "Shelf.ply";
            transform = transl(2.40, -1.5, 1.2) * trotx(0) * troty(0) * trotz(deg2rad(90));
            self.insertObj(file, transform);
           
            file = "Shelf.ply";
            transform = transl(-2, -1.25, 1.4) * trotx(0) * troty(0) * trotz(deg2rad(90));
            self.insertObj(file, transform);
        
            % Insert Oven
            file = "Oven.ply";
            transform = transl(-1.75, -1.25, 0.9) * trotx(0) * troty(0) * trotz(deg2rad(90));
            self.insertObj(file, transform);
        
            % Insert Coffee Machine
            file = "Coffee.ply";
            transform = transl(-0.5, -0.25, 1.1) * trotx(0) * troty(0) * trotz(deg2rad(0));
            self.insertObj(file, transform);
        
            % Insert Dining Table
            file = "Dining Table.ply";
            transform = transl(0, 2.5, 0.5) * trotx(0) * troty(0) * trotz(deg2rad(0));
            self.insertObj(file, transform);

            % Insert Washing Basket
            PlaceObject('Basket.PLY', [-1.15, 2.5, 0.67]);

            % Insert Fire Extinguisher
            PlaceObject('Extinguish.PLY', [-1.75, 0, 0]);

            % Insert Sprinklers
            PlaceObject('Sprinkler.PLY', [-1, 2.5, 2.4]);
            PlaceObject('Sprinkler.PLY', [1, 2.5, 2.4]);
            PlaceObject('Sprinkler.PLY', [-1, -0.5, 2.4]);
            PlaceObject('Sprinkler.PLY', [0, -0.5, 2.4]);
            PlaceObject('Sprinkler.PLY', [1, -0.5, 2.4]);

            % Insert MedKit
            PlaceObject('MedKit.PLY', [2.5, -1.5, 1.19]);

        end

        function insertObj(self, name, transform)
            [f,v,data] = plyread(name,'tri');
            VertCount = size(v,1);
            midPoint = sum(v)/VertCount;
            Verts = v - repmat(midPoint,VertCount,1);
            Pose = eye(4);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            Mesh_h = trisurf(f,Verts(:,1),Verts(:,2), Verts(:,3) ...
                           ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat'); % EdgeColor = interp for meshes
            Pose = Pose * transform; % Location of Object
            updatedPoints = (Pose * [Verts,ones(VertCount,1)]')';
            Mesh_h.Vertices = updatedPoints(:,1:3);
        end
    end

    methods(Static)
        function fig()
            camlight
            axis on
            grid on
            view(135, 30) % Rotate plane (z direction) by 135deg & look down at 30deg
            axis([-4 4 -4 4 0 3]);
            xticks(-5:0.5:5);
            xlabel('x');
            yticks(-5:0.5:5);
            ylabel('y');
            zticks(0:0.1:3);
            zlabel('z');
            daspect([1 1 1])
            hold on
        end
    end
end

