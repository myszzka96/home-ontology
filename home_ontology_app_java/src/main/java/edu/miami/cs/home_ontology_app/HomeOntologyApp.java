package edu.miami.cs.home_ontology_app;

import java.util.Iterator;

import com.github.owlcs.ontapi.internal.AxiomParserProvider;
import com.github.owlcs.ontapi.jena.OntModelFactory;
import com.github.owlcs.ontapi.OntManagers;
import com.github.owlcs.ontapi.OntologyManager;
import com.github.owlcs.ontapi.Ontology;

import org.apache.jena.query.QueryExecution;
import org.apache.jena.query.QueryExecutionFactory;
import org.apache.jena.query.QueryFactory;
import org.apache.jena.query.QuerySolution;
import org.apache.jena.query.ResultSet;
import org.apache.jena.update.UpdateAction;
import org.apache.jena.rdf.model.Literal;
import org.apache.jena.rdf.model.Model;
import org.apache.jena.rdf.model.Resource;

import org.semanticweb.owlapi.model.AxiomType;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLAnnotationProperty;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;

/**
 * Represents the main class for the home ontology application.
 *
 * This application listens to requests from other applications for information in
 * the ontology. The request consists of keywords, which this application uses 
 * to generate a SQARQL query and then proceeds to find triples relevant to the 
 * keywords. It returns the triples as a list of tuples, where each tuple consists
 * of three strings representing (subject, predicate, object).
 */
public class HomeOntologyApp
{
    /*
     * Try to load ontology based on its International Resource Identifier (IRI).
     * To use ontapi interface between owlapi and apache jena, we need to cast
     * an instance of owlapi.model.OWLOntology into owlcs.ontapi.Ontology.
     *
     * TODO: load ontology from a flat file in a /resources folder so that
     * we can get ontology even if there is no internet connection.
     */
    public static Ontology getOntology(OntologyManager manager, IRI iri)
    {
        Ontology ontology = null;
        try {
            ontology = (Ontology) manager.loadOntology(iri);
        }
        catch (OWLOntologyCreationException e) {
            e.printStackTrace();
        }

        return ontology;
    }

    /*
     * Example of an input triple result from SPARQL query, broken up into three lines for readability:
     *   ( ?subject = <http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#Orange> )
     *   ( ?predicate = <http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#isLocatedInsideHomeEquipment> )
     *   ( ?object = <http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#Fridge> )
     *
     * After iterating through the variable names, getting the Resource object associated with
     * each variable, and casting the Resource object into a String, we get the following:
     *   http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#Orange
     *   http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#isLocatedInsideHomeEquipment
     *   http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#Fridge
     *
     * We want this to be the final output of the function
     *   Orange isLocatedInsideHomeEquipment Fridge
     *
     * Notice that each element of the triple is preceded by a '#' symbol. As long as the website
     * has no other occurrences of the '#' symbol, we can use it as a delimiter. For safety, we can
     * specify to only use the last '#' as a starting delimiter.
     *
     * NOTE: there may be exceptional cases where this logic fails, but for now we use this simple parse.
     * From the Jena documentation, it seems there is no built-in parser that strips out the IRI
     * associated with the resource for us, so I have to do this myself.
     */
    public static String parseSolutionToTriple(QuerySolution solution)
    {
        String triple = "";
        char startDelimiter = '#';

        // Iterate through the variable names; in this case we have three variable names and they are:
        // subject, predicate, object. For each variable name, get the String value associated with it.
        Iterator<String> iterator = solution.varNames();
        while (iterator.hasNext())
        {
            String varName = iterator.next();
            String value = solution.getResource(varName).toString();

            // If any of the three variable names has a value that does not contain '#' character,
            // we return a blank String which means no triple.
            int startIndex = value.lastIndexOf(startDelimiter);
            if (startIndex == -1) {
                return "";
            }

            triple += value.substring(startIndex + 1) + " ";
        }

        return triple;
    }

    public static void main(String[] args) throws Exception
    {
        /*
         * Dedicated argument parser is too fancy for now. Instead, we assume the first argument
         * is the name of the class that we wish to use as the subject of the SPARQL query.
         * The command to execute the .jar file of this application will look like this:
         *
         *   javac -jar target/home_ontology_app-0.0.1.jar <subject_name>
         */
        String subjectString = args[0];

        // Initialize ont-api OntologyManager which extends owl-api OWLOntologyManager.
        OntologyManager manager = OntManagers.createONT();
        String iriString = "https://web.cs.miami.edu/home/kpas751/house_onto.owl";
        IRI iri = IRI.create(iriString);
        Ontology ontology = getOntology(manager, iri);

        assert ontology != null;

        /* Description of SPARQL query steps:
         *
         * 1) Specify that the subject muse be an individual, and not a class or property.
         *
         * 2) For now we are querying instances of the class Fruit. We can't directly specify
         *    an individual that we already know, such as orange. We could only do this if we
         *    gave the individual orange a data property called name, with the value "orange".
         *
         * 3) For now we need specify that the predicate is an object property.
         *    In our case, isLocatedInsideHomeEquipment. It seems that RDF format cannot give
         *    us the inferred triples. For example, we have declared in the ontology that
         *      "orange isLocatedInsideHomeEquipment fridge"
         *      "isLocatedInsideHomeEquipment rdfs:subPropertyOf isLocatedInHomeEquipment"
         *      "isLocatedInHomeEquipment rdfs:subProperty isLocatedIn"
         *
         *    However, we cannot get the inferred triples:
         *      "orange isLocatedInHomeEquipment fridge"
         *      "orange isLocatedIn fridge"
         *
         * NOTE: by default the namespace of the RDF document as converted by ont-api is
         *   <http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#>.
         * However, the actual owl file is stored on a server at web.cs.miami.edu:
         *   <https://web.cs.miami.edu/home/kpas751/house_onto#>.
         */
        StringBuilder queryBuilder = new StringBuilder(10);
        queryBuilder.append("PREFIX owl:   <http://www.w3.org/2002/07/owl#> \n");
        queryBuilder.append("PREFIX rdf:   <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \n");
        queryBuilder.append("PREFIX home:  <http://www.semanticweb.org/kasia/ontologies/2020/3/house-onto#> \n");
        queryBuilder.append("SELECT ?subject ?predicate ?object \n");
        queryBuilder.append("WHERE {\n");
        queryBuilder.append("    ?subject ?predicate ?object . \n");
        queryBuilder.append("    ?subject rdf:type owl:NamedIndividual . \n");
        queryBuilder.append("    ?subject rdf:type home:" + subjectString + " . \n");
        queryBuilder.append("    ?predicate rdf:type owl:ObjectProperty . \n");
        queryBuilder.append("}");

        // Convert Ontology object into a Jena RDF Model object so we can execute SPARQL queries on it.
        try (QueryExecution qexec = QueryExecutionFactory.create(
                QueryFactory.create(queryBuilder.toString()), ontology.asGraphModel())
            )
        {
            ResultSet res = qexec.execSelect();
            // Convert each QueryResult into a triple, where each element in the triple has
            // its IRI removed. The formatted triple will be printed and the output can be
            // piped to a text file. If no triple was found, output is an empty string.
            while (res.hasNext()) {
                String triple = parseSolutionToTriple(res.next());
                System.out.println(triple);
            }
        }
    }
}
