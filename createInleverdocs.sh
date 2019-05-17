#copy pdfs to directory
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
cp ./Docs/Plan\ van\ aanpak/Plan\ van\ aanpak.pdf ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
cp ./Docs/Afstudeerverslag/Afstudeerverslag/Afstudeerverslag.pdf ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
cp ./Docs/DesignDocumenten/SoftwareDesignDocument.pdf ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
cp ./Docs/DesignDocumenten/SoftwareRequirementSpecification.pdf ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
cp ./Docs/DesignDocumenten/SoftwareRequirementSpecification.pdf ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
cp ./Docs/Onderzoeken/DroneMeshnetwerkSimulatie/Onderzoeksrapport\ Drone\ meshnetwerk\ simulatie.pdf ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/
#copy templates to directory
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/Templates
cp -R ./Docs/Plan\ van\ aanpak/Templates/* ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/Templates/
#copy bijlagen to directory
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/Bijlagen
cp -R ./Docs/Onderzoeken/DroneMeshnetwerkSimulatie/Bijlagen/* ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Docs/Bijlagen/
#copy code to directory
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Code/
cp -R ./Code/ros/src/* ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Code/
rm -r ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Code/drone_meshnetwork_simulation/build/
rm -r ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Code/drone_meshnetwork_simulation/.vscode
rm ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Code/drone_meshnetwork_simulation/.clang-format
rm ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Code/drone_meshnetwork_simulation_workspace.code-workspace
#copy astah diagram
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/astah/
cp ./Astah/Ontwerpen.asta ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/astah/
#copy plantuml source diagrams
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/plantuml/source 
cp -R ./Docs/DesignDocumenten/UML/* ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/plantuml/source
rm -r ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/plantuml/source/out
#copy plantuml image diagrams
mkdir -p ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/plantuml/image 
cp -R ./Docs/DesignDocumenten/UML/out/* ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten/Ontwerpen/plantuml/image

#zip it and ship it
zip -r Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten.zip ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten
zip -r Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten_zonder_mp4.zip ./Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten -x "*.mp4"
echo "  "
echo "Created Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten.zip at path:"
readlink -f Scriptie_Maurice_Berentsen_561399_Drone_Meshnetwerk_Alten.zip 
echo "nou je hebt het geflikt upload die zip op isas en ga bier drinken ofzo"
